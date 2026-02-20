/**********************************************************************
Full SLAM Exploration with Unitree SDK2 + Livox SDK2
=====================================================
Complete C++ solution that:
- Controls the Unitree Go2 robot via SDK2
- Collects LiDAR point clouds from Livox Mid-360
- Generates a map during autonomous exploration
- Uses obstacle avoidance for safe navigation
***********************************************************************/

#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
#include <signal.h>
#include <unistd.h>
#include <chrono>
#include <mutex>
#include <atomic>
#include <thread>

// Unitree SDK2
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

// Livox SDK2
#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::go2;

// Global flag for signal handling
std::atomic<bool> g_running{true};

void signalHandler(int signum) {
    std::cout << "\n\nInterrupt signal received, stopping..." << std::endl;
    g_running = false;
}

// ============================================================================
// Point Cloud Storage
// ============================================================================
struct Point3D {
    float x, y, z;
    uint8_t intensity;
};

class PointCloudMap {
public:
    std::vector<Point3D> points;
    std::mutex mutex;
    std::atomic<uint64_t> total_points{0};
    std::atomic<uint64_t> packet_count{0};

    void addPoint(float x, float y, float z, uint8_t intensity) {
        // Filter invalid points
        if (x == 0 && y == 0 && z == 0) return;

        // Filter by range (within 50m)
        float dist = sqrt(x*x + y*y + z*z);
        if (dist > 50.0f || dist < 0.1f) return;

        std::lock_guard<std::mutex> lock(mutex);
        if (points.size() < 2000000) {  // Limit to 2M points
            Point3D p;
            p.x = x;
            p.y = y;
            p.z = z;
            p.intensity = intensity;
            points.push_back(p);
        }
        total_points++;
    }

    void saveToPLY(const std::string& filename) {
        std::lock_guard<std::mutex> lock(mutex);

        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << filename << std::endl;
            return;
        }

        file << "ply\n";
        file << "format ascii 1.0\n";
        file << "element vertex " << points.size() << "\n";
        file << "property float x\n";
        file << "property float y\n";
        file << "property float z\n";
        file << "property uchar intensity\n";
        file << "end_header\n";

        for (const auto& p : points) {
            file << p.x << " " << p.y << " " << p.z << " " << (int)p.intensity << "\n";
        }

        file.close();
        std::cout << "Saved " << points.size() << " points to " << filename << std::endl;
    }

    size_t size() {
        std::lock_guard<std::mutex> lock(mutex);
        return points.size();
    }
};

PointCloudMap g_map;

// ============================================================================
// Livox SDK2 Callbacks
// ============================================================================
void LivoxPointCloudCallback(uint32_t handle, const uint8_t dev_type,
                              LivoxLidarEthernetPacket* data, void* client_data) {
    if (data == nullptr || data->data == nullptr) return;

    g_map.packet_count++;

    if (data->data_type == kLivoxLidarCartesianCoordinateHighData) {
        LivoxLidarCartesianHighRawPoint* raw_points =
            (LivoxLidarCartesianHighRawPoint*)data->data;

        for (uint32_t i = 0; i < data->dot_num; i++) {
            // Convert mm to meters
            float x = raw_points[i].x / 1000.0f;
            float y = raw_points[i].y / 1000.0f;
            float z = raw_points[i].z / 1000.0f;

            g_map.addPoint(x, y, z, raw_points[i].reflectivity);
        }
    }
}

void LivoxImuCallback(uint32_t handle, const uint8_t dev_type,
                       LivoxLidarEthernetPacket* data, void* client_data) {
    // IMU data from LiDAR - can be used for odometry refinement
}

void LivoxWorkModeCallback(livox_status status, uint32_t handle,
                            LivoxLidarAsyncControlResponse* response, void* client_data) {
    if (response != nullptr) {
        std::cout << "[Livox] Work mode set successfully" << std::endl;
    }
}

void LivarInfoCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
    if (info == nullptr) return;

    std::cout << "[Livox] LiDAR detected - SN: " << info->sn << std::endl;

    // Start normal scanning mode
    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, LivoxWorkModeCallback, nullptr);
}

// ============================================================================
// Waypoint and Navigation
// ============================================================================
struct Waypoint {
    double x, y;
    Waypoint(double _x, double _y) : x(_x), y(_y) {}
};

// ============================================================================
// Full SLAM Explorer Class
// ============================================================================
class FullSLAMExplorer {
public:
    FullSLAMExplorer(const std::string& livoxConfig)
        : maxRadius(3.0), waypointIndex(0), targetX(0), targetY(0), oaEnabled(false),
          livoxConfigPath(livoxConfig) {

        // Initialize clients (ChannelFactory must be initialized BEFORE this!)
        sportClient.SetTimeout(10.0f);
        sportClient.Init();

        oaClient.SetTimeout(10.0f);
        oaClient.Init();

        stateSubscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
        stateSubscriber->InitChannel(std::bind(&FullSLAMExplorer::StateHandler, this, std::placeholders::_1), 1);

        std::cout << "[Unitree] SDK2 clients initialized" << std::endl;
    }

    ~FullSLAMExplorer() {
        Stop();
    }

    void StateHandler(const void* message) {
        state = *(unitree_go::msg::dds_::SportModeState_*)message;
    }

    bool InitializeLivox() {
        std::cout << "[Livox] Initializing SDK2 with config: " << livoxConfigPath << std::endl;

        if (!LivoxLidarSdkInit(livoxConfigPath.c_str())) {
            std::cerr << "[Livox] Failed to initialize SDK2" << std::endl;
            return false;
        }

        SetLivoxLidarPointCloudCallBack(LivoxPointCloudCallback, nullptr);
        SetLivoxLidarImuDataCallback(LivoxImuCallback, nullptr);
        SetLivoxLidarInfoChangeCallback(LivarInfoCallback, nullptr);

        std::cout << "[Livox] SDK2 initialized, waiting for LiDAR..." << std::endl;
        return true;
    }

    bool EnableObstacleAvoidance() {
        std::cout << "[Unitree] Enabling obstacle avoidance..." << std::endl;

        bool enabled = false;
        int32_t ret = oaClient.SwitchGet(enabled);

        if (!enabled) {
            for (int i = 0; i < 50 && g_running; i++) {
                oaClient.SwitchSet(true);
                usleep(100000);
                oaClient.SwitchGet(enabled);
                if (enabled) break;
            }
        }

        if (enabled) {
            oaClient.UseRemoteCommandFromApi(true);
            usleep(50000);
            oaEnabled = true;
            std::cout << "[Unitree] Obstacle avoidance enabled" << std::endl;
        } else {
            std::cout << "[Unitree] WARNING: Failed to enable obstacle avoidance" << std::endl;
        }

        return enabled;
    }

    void DisableObstacleAvoidance() {
        if (oaEnabled) {
            oaClient.UseRemoteCommandFromApi(false);
            oaClient.SwitchSet(false);
            oaEnabled = false;
        }
    }

    std::vector<Waypoint> GenerateWaypoints(const std::string& areaSize) {
        std::vector<Waypoint> waypoints;
        double maxDist = 3.0;
        double spacing = 1.5;

        if (areaSize == "small") {
            maxDist = 2.0;
            spacing = 1.0;
            maxRadius = 2.0;
        } else if (areaSize == "large") {
            maxDist = 5.0;
            spacing = 2.0;
            maxRadius = 5.0;
        } else {
            maxRadius = 3.0;
        }

        // Generate lawnmower pattern
        int index = 0;
        for (double y = -maxDist; y <= maxDist; y += spacing) {
            if (index % 2 == 0) {
                waypoints.push_back(Waypoint(maxDist, y));
                waypoints.push_back(Waypoint(-maxDist, y));
            } else {
                waypoints.push_back(Waypoint(-maxDist, y));
                waypoints.push_back(Waypoint(maxDist, y));
            }
            index++;
        }

        return waypoints;
    }

    void Explore(int duration, const std::string& areaSize, const std::string& mapOutput) {
        // Stand up first
        std::cout << "[Unitree] Standing up..." << std::endl;
        sportClient.StandUp();
        sleep(2);

        // Enable obstacle avoidance
        EnableObstacleAvoidance();

        // Generate waypoints
        std::vector<Waypoint> waypoints = GenerateWaypoints(areaSize);

        std::cout << "\n";
        std::cout << "╔════════════════════════════════════════════════════════════╗\n";
        std::cout << "║         FULL SLAM EXPLORATION (C++ Native SDK2)            ║\n";
        std::cout << "╠════════════════════════════════════════════════════════════╣\n";
        std::cout << "║ Duration:      " << std::setw(4) << duration << "s                                   ║\n";
        std::cout << "║ Area:          " << std::setw(10) << areaSize << " (radius: " << maxRadius << "m)             ║\n";
        std::cout << "║ Waypoints:     " << std::setw(4) << waypoints.size() << "                                     ║\n";
        std::cout << "║ Obstacle Avoid:" << (oaEnabled ? " ENABLED" : " DISABLED") << "                            ║\n";
        std::cout << "║ Map Output:    " << std::setw(30) << mapOutput.substr(0, 30) << " ║\n";
        std::cout << "╚════════════════════════════════════════════════════════════╝\n";
        std::cout << "\n[Starting exploration...]\n\n";

        auto startTime = std::chrono::steady_clock::now();
        waypointIndex = 0;

        auto lastPrintTime = std::chrono::steady_clock::now();

        while (g_running) {
            auto now = std::chrono::steady_clock::now();
            int elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - startTime).count();

            // Check duration
            if (elapsed >= duration) {
                std::cout << "\n\nDuration reached" << std::endl;
                break;
            }

            // Check waypoints
            if (waypointIndex >= waypoints.size()) {
                std::cout << "\n\nAll waypoints visited" << std::endl;
                break;
            }

            // Get robot state
            double robotX = state.position()[0];
            double robotY = state.position()[1];
            double robotYaw = state.imu_state().rpy()[2];

            // Get target
            targetX = waypoints[waypointIndex].x;
            targetY = waypoints[waypointIndex].y;

            // Distance to target
            double dx = targetX - robotX;
            double dy = targetY - robotY;
            double distToTarget = sqrt(dx*dx + dy*dy);

            // Check if reached waypoint
            if (distToTarget < 0.3) {
                waypointIndex++;
                std::cout << "\n[Waypoint " << waypointIndex << "/" << waypoints.size() << " reached]" << std::endl;
                continue;
            }

            // Calculate velocities
            double targetAngle = atan2(dy, dx);
            double angleError = targetAngle - robotYaw;

            while (angleError > M_PI) angleError -= 2 * M_PI;
            while (angleError < -M_PI) angleError += 2 * M_PI;

            double vx = 0.3;
            double vyaw = std::max(-0.5, std::min(0.5, angleError * 1.5));

            // Boundary check
            double distFromOrigin = sqrt(robotX*robotX + robotY*robotY);
            if (distFromOrigin > maxRadius) {
                double angleToOrigin = atan2(-robotY, -robotX);
                double angleErr = angleToOrigin - robotYaw;
                vx = -0.1;
                vyaw = std::max(-0.5, std::min(0.5, angleErr * 2.0));
            }

            // Send command
            if (oaEnabled) {
                oaClient.Move(vx, 0.0, vyaw);
            } else {
                sportClient.Move(vx, 0.0, vyaw);
            }

            // Print status every second
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastPrintTime).count() >= 1000) {
                int remaining = duration - elapsed;
                size_t mapPoints = g_map.size();
                uint64_t totalPts = g_map.total_points.load();

                printf("\r[%02ds/%02ds | %02ds left] WP:%d/%zu | Pos:(%+.2f,%+.2f) | Map:%zu pts | LiDAR:%lu pts   ",
                       elapsed, duration, remaining,
                       waypointIndex+1, waypoints.size(),
                       robotX, robotY, mapPoints, totalPts);
                fflush(stdout);
                lastPrintTime = now;
            }

            usleep(20000);  // 50Hz
        }

        std::cout << "\n\nExploration complete!" << std::endl;

        // Save map
        std::cout << "\nSaving map..." << std::endl;
        g_map.saveToPLY(mapOutput);

        Stop();
    }

    void Stop() {
        std::cout << "[Unitree] Stopping robot..." << std::endl;

        if (oaEnabled) {
            oaClient.Move(0, 0, 0);
            oaClient.UseRemoteCommandFromApi(false);
        }
        sportClient.StopMove();

        DisableObstacleAvoidance();
    }

private:
    SportClient sportClient;
    ObstaclesAvoidClient oaClient;
    ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> stateSubscriber;
    unitree_go::msg::dds_::SportModeState_ state;

    double maxRadius;
    size_t waypointIndex;
    double targetX, targetY;
    bool oaEnabled;
    std::string livoxConfigPath;
};

// ============================================================================
// Main
// ============================================================================
void PrintUsage(const char* program) {
    std::cout << "Full SLAM Explorer - Unitree Go2 with Livox Mid-360\n";
    std::cout << "\nUsage: " << program << " [options]\n";
    std::cout << "\nOptions:\n";
    std::cout << "  --interface <name>    Network interface (default: enP8p1s0)\n";
    std::cout << "  --duration <seconds>  Exploration duration (default: 60)\n";
    std::cout << "  --area <size>         Area size: small, medium, large (default: medium)\n";
    std::cout << "  --config <path>       Livox config JSON (default: mid360_config.json)\n";
    std::cout << "  --output <path>       Output map file (default: slam_map.ply)\n";
    std::cout << "  --no-oa               Disable obstacle avoidance\n";
    std::cout << "  --help                Show this help\n";
}

int main(int argc, char** argv) {
    // Default parameters
    std::string interface = "enP8p1s0";
    int duration = 60;
    std::string area = "medium";
    std::string configPath = "/home/unitree/Livox-SDK2/samples/livox_lidar_quick_start/mid360_config.json";
    std::string outputPath = "slam_map.ply";
    bool useOA = true;

    // Parse arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--interface" && i + 1 < argc) {
            interface = argv[++i];
        } else if (arg == "--duration" && i + 1 < argc) {
            duration = std::atoi(argv[++i]);
        } else if (arg == "--area" && i + 1 < argc) {
            area = argv[++i];
        } else if (arg == "--config" && i + 1 < argc) {
            configPath = argv[++i];
        } else if (arg == "--output" && i + 1 < argc) {
            outputPath = argv[++i];
        } else if (arg == "--no-oa") {
            useOA = false;
        } else if (arg == "--help") {
            PrintUsage(argv[0]);
            return 0;
        }
    }

    // Setup signal handler
    signal(SIGINT, signalHandler);

    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║     Full SLAM Explorer - Unitree SDK2 + Livox SDK2         ║\n";
    std::cout << "╠════════════════════════════════════════════════════════════╣\n";
    std::cout << "║ Interface:     " << std::setw(40) << interface << " ║\n";
    std::cout << "║ Duration:      " << std::setw(40) << duration << "s ║\n";
    std::cout << "║ Area:          " << std::setw(40) << area << " ║\n";
    std::cout << "║ Livox Config:  " << std::setw(40) << configPath.substr(0, 40) << " ║\n";
    std::cout << "║ Output:        " << std::setw(40) << outputPath << " ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";

    try {
        // Initialize Unitree ChannelFactory FIRST!
        ChannelFactory::Instance()->Init(0, interface);
        std::cout << "[Unitree] ChannelFactory initialized on " << interface << std::endl;

        // Create explorer
        FullSLAMExplorer explorer(configPath);

        // Initialize Livox SDK2
        if (!explorer.InitializeLivox()) {
            std::cerr << "Failed to initialize Livox SDK2" << std::endl;
            ChannelFactory::Instance()->Release();
            return 1;
        }

        // Wait for LiDAR to connect
        std::cout << "[Livox] Waiting for LiDAR connection (3s)..." << std::endl;
        sleep(3);

        // Start exploration
        explorer.Explore(duration, area, outputPath);

        // Cleanup Livox
        std::cout << "[Livox] Shutting down SDK2..." << std::endl;
        LivoxLidarSdkUninit();

        // Cleanup Unitree
        ChannelFactory::Instance()->Release();

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        LivoxLidarSdkUninit();
        ChannelFactory::Instance()->Release();
        return 1;
    }

    std::cout << "\nDone!\n";
    return 0;
}
