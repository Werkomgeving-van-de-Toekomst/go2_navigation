/**********************************************************************
SLAM Exploration with Obstacle Avoidance for Unitree Go2
Using native C++ SDK2 with RealSense camera integration
All real-time control in C++ - no Python dependencies
***********************************************************************/

#include <cmath>
#include <vector>
#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

// RealSense C++ SDK
#include <librealsense2/rs.hpp>

#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::go2;

// Global flag for signal handling
volatile bool running = true;

void signalHandler(int signum) {
    std::cout << "\n\nInterrupt signal received, stopping..." << std::endl;
    running = false;
}

// Waypoint structure
struct Waypoint {
    double x;
    double y;
    Waypoint(double _x, double _y) : x(_x), y(_y) {}
};

// Camera obstacle data
struct CameraObstacles {
    float front = 999.0f;
    float left = 999.0f;
    float right = 999.0f;
    float rear = 999.0f;
    bool frontBlocked = false;
    bool leftBlocked = false;
    bool rightBlocked = false;
    bool valid = false;
};

// RealSense Camera Thread
class RealSenseCamera {
private:
    rs2::pipeline pipeline;
    rs2::align align_to_color;
    std::atomic<bool> cameraRunning{false};
    std::atomic<bool> cameraAvailable{false};
    std::thread cameraThread;
    std::mutex dataMutex;
    CameraObstacles obstacles;

    int width = 424;
    int height = 240;
    float frontThreshold = 0.30f;   // 30cm - original
    float sideThreshold = 0.25f;    // 25cm - original

public:
    RealSenseCamera() : align_to_color(RS2_STREAM_COLOR) {}

    bool init() {
        try {
            rs2::config config;
            // Use default resolution - let the camera choose
            config.enable_stream(RS2_STREAM_DEPTH, RS2_FORMAT_Z16);
            config.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_BGR8);

            auto profile = pipeline.start(config);

            // Get actual stream dimensions
            auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
            width = depth_stream.width();
            height = depth_stream.height();

            std::cout << "[Camera] Resolution: " << width << "x" << height << std::endl;

            cameraAvailable = true;

            // Warm up camera
            for (int i = 0; i < 30; i++) {
                auto frames = pipeline.wait_for_frames();
            }

            std::cout << "[Camera] RealSense initialized" << std::endl;
            return true;
        } catch (const rs2::error& e) {
            std::cout << "[Camera] RealSense not available: " << e.what() << std::endl;
            cameraAvailable = false;
            return false;
        }
    }

    void start() {
        if (!cameraAvailable) return;
        cameraRunning = true;
        cameraThread = std::thread(&RealSenseCamera::run, this);
    }

    void stop() {
        cameraRunning = false;
        if (cameraThread.joinable()) {
            cameraThread.join();
        }
        if (cameraAvailable) {
            try {
                pipeline.stop();
            } catch (...) {}
        }
    }

    CameraObstacles getObstacles() {
        std::lock_guard<std::mutex> lock(dataMutex);
        return obstacles;
    }

    bool isAvailable() const { return cameraAvailable; }

private:
    void run() {
        while (cameraRunning && running) {
            try {
                auto frames = pipeline.wait_for_frames(100);  // 100ms timeout
                auto depth = align_to_color.process(frames).get_depth_frame();

                if (!depth) continue;

                CameraObstacles newObs;
                newObs.valid = true;

                int thirdW = width / 3;
                int bottomThird = height * 2 / 3;  // Lower 2/3 of image

                // Get depths for each region
                newObs.front = getRegionDepth(depth, thirdW, 2*thirdW, bottomThird, height);
                newObs.left = getRegionDepth(depth, 0, thirdW, bottomThird, height);
                newObs.right = getRegionDepth(depth, 2*thirdW, width, bottomThird, height);

                // Check if blocked
                newObs.frontBlocked = newObs.front < frontThreshold;
                newObs.leftBlocked = newObs.left < sideThreshold;
                newObs.rightBlocked = newObs.right < sideThreshold;

                // Update shared data
                {
                    std::lock_guard<std::mutex> lock(dataMutex);
                    obstacles = newObs;
                }
            } catch (const rs2::error& e) {
                // Camera error, continue trying
            }
        }
    }

    float getRegionDepth(const rs2::depth_frame& depth, int x1, int x2, int y1, int y2) {
        std::vector<float> depths;

        // Sample every 8th pixel for speed
        for (int y = y1; y < y2; y += 8) {
            for (int x = x1; x < x2; x += 8) {
                float d = depth.get_distance(x, y);
                if (d > 0.1f && d < 10.0f) {
                    depths.push_back(d);
                }
            }
        }

        if (depths.size() < 10) return 999.0f;

        // Return median
        std::sort(depths.begin(), depths.end());
        return depths[depths.size() / 2];
    }
};

class SLAMExplorer {
public:
    SLAMExplorer()
        : maxRadius(3.0), waypointIndex(0), targetX(0), targetY(0), oaEnabled(false) {

        sportClient.SetTimeout(10.0f);
        sportClient.Init();

        oaClient.SetTimeout(10.0f);
        oaClient.Init();

        stateSubscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
        stateSubscriber->InitChannel(std::bind(&SLAMExplorer::StateHandler, this, std::placeholders::_1), 1);

        std::cout << "Clients initialized" << std::endl;
    }

    ~SLAMExplorer() {
        Stop();
        camera.stop();
    }

    void StateHandler(const void* message) {
        state = *(unitree_go::msg::dds_::SportModeState_*)message;
    }

    float getFrontObstacle() { return state.range_obstacle()[0]; }
    float getRightObstacle() { return state.range_obstacle()[1]; }
    float getRearObstacle() { return state.range_obstacle()[2]; }
    float getLeftObstacle() { return state.range_obstacle()[3]; }

    bool EnableObstacleAvoidance() {
        std::cout << "Enabling obstacle avoidance..." << std::endl;

        bool enabled = false;
        int32_t ret = oaClient.SwitchGet(enabled);

        if (!enabled) {
            for (int i = 0; i < 50 && running; i++) {
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
            std::cout << "Obstacle avoidance enabled" << std::endl;
        } else {
            std::cout << "WARNING: Failed to enable obstacle avoidance" << std::endl;
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

    void Explore(int duration, const std::string& areaSize, bool useCamera) {
        // Initialize camera if requested
        if (useCamera) {
            if (camera.init()) {
                camera.start();
                std::cout << "Camera obstacle detection enabled (C++ native)" << std::endl;
            } else {
                std::cout << "Camera not available, continuing without" << std::endl;
            }
        }

        // Stand up and balance
        std::cout << "Standing up..." << std::endl;
        sportClient.StandUp();
        sleep(3);

        std::cout << "Balance stand..." << std::endl;
        sportClient.BalanceStand();
        sleep(3);

        // Enable obstacle avoidance
        EnableObstacleAvoidance();
        sleep(1);

        std::vector<Waypoint> waypoints = GenerateWaypoints(areaSize);

        std::cout << "\n============================================================" << std::endl;
        std::cout << "SLAM EXPLORATION - WANDER MODE" << std::endl;
        std::cout << "============================================================" << std::endl;
        std::cout << "Duration: " << duration << "s | Area: " << areaSize << std::endl;
        std::cout << "Camera: " << (camera.isAvailable() ? "yes" : "no") << std::endl;
        std::cout << "============================================================\n" << std::endl;

        // Velocity
        double currentVx = 0, currentVy = 0, currentVyaw = 0;

        // Stuck detection
        double lastX = 0, lastY = 0;
        int stuckCount = 0;
        int escapePhase = 0;
        int escapeTimer = 0;
        int turnDirection = 1;
        int totalEscapes = 0;

        // Random wander direction
        double wanderYaw = 0;
        int wanderTimer = 0;

        time_t startTime = time(nullptr);

        while (running && (time(nullptr) - startTime) < duration) {
            double robotX = state.position()[0];
            double robotY = state.position()[1];
            double robotYaw = state.imu_state().rpy()[2];

            // Get camera data
            CameraObstacles camObs;
            if (camera.isAvailable()) {
                camObs = camera.getObstacles();
            }

            // Update wander direction periodically
            wanderTimer--;
            if (wanderTimer <= 0) {
                wanderYaw = robotYaw + ((rand() % 3) - 1) * 0.5;  // Small random adjustment
                wanderTimer = 50;  // Every second
            }

            double targetVx = 0.40;
            double targetVy = 0;
            double targetVyaw = 0;

            // Obstacle avoidance - only when close
            if (camObs.valid) {
                // Only turn when really close
                if (camObs.front < 0.40f) {  // 40cm
                    targetVx = 0.15;
                    targetVyaw = (camObs.left > camObs.right) ? 0.6 : -0.6;
                }
                if (camObs.front < 0.30f) {  // 30cm - very close
                    targetVx = 0;
                    targetVyaw = (camObs.left > camObs.right) ? 0.8 : -0.8;
                }
                // Sidestep only when very close
                if (camObs.left < 0.25f) targetVy = -0.25;
                else if (camObs.right < 0.25f) targetVy = 0.25;
            } else {
                // No camera - just wander
                double yawError = wanderYaw - robotYaw;
                while (yawError > M_PI) yawError -= 2 * M_PI;
                while (yawError < -M_PI) yawError += 2 * M_PI;
                targetVyaw = yawError * 0.5;
            }

            // Stuck detection - less sensitive
            double moved = sqrt((robotX - lastX)*(robotX - lastX) + (robotY - lastY)*(robotY - lastY));
            if (escapePhase == 0) {
                if (moved < 0.01 && targetVx > 0.3) {  // Only if really not moving
                    stuckCount++;
                } else if (moved > 0.02) {
                    stuckCount = std::max(0, stuckCount - 5);
                }
            }

            // Escape only when really stuck for longer
            if (stuckCount > 100 && escapePhase == 0) {  // 2 seconds of being truly stuck
                totalEscapes++;
                turnDirection = (rand() % 2 == 0) ? 1 : -1;
                escapePhase = 1;
                escapeTimer = 30;  // Shorter escape
                std::cout << "\n[STUCK! Escape " << totalEscapes << "]" << std::endl;
            }

            if (escapePhase > 0) {
                escapeTimer--;
                if (escapePhase == 1) {
                    // Quick reverse
                    currentVx = -0.30;
                    currentVy = 0;
                    currentVyaw = 0.5 * turnDirection;
                    if (escapeTimer <= 0) {
                        escapePhase = 2;
                        escapeTimer = 30;
                    }
                } else {
                    // Turn in place
                    currentVx = 0;
                    currentVy = 0;
                    currentVyaw = 0.9 * turnDirection;
                    if (escapeTimer <= 0) {
                        escapePhase = 0;
                        stuckCount = 0;
                        wanderTimer = 0;
                    }
                }
            } else {
                // Normal movement
                currentVx = targetVx;
                currentVy = targetVy;
                currentVyaw = targetVyaw;
            }

            // Send command
            if (oaEnabled) {
                oaClient.Move(currentVx, currentVy, currentVyaw);
            } else {
                sportClient.Move(currentVx, currentVy, currentVyaw);
            }

            // Print status
            static time_t lastPrint = 0;
            if (time(nullptr) - lastPrint >= 1) {
                int elapsed = time(nullptr) - startTime;
                const char* status = (escapePhase == 1) ? " [REV]" : (escapePhase == 2) ? " [TURN]" : "";
                printf("\r[%02ds/%02ds] Pos:(%+.1f,%+.1f) | Vel:(%.2f,%.2f,%.2f) | Cam:(F%.2f,L%.2f,R%.2f)%s  ",
                       elapsed, duration, robotX, robotY, currentVx, currentVy, currentVyaw,
                       camObs.front, camObs.left, camObs.right, status);
                fflush(stdout);
                lastPrint = time(nullptr);
            }

            lastX = robotX;
            lastY = robotY;
            usleep(20000);
        }

        std::cout << "\n\nExploration complete!" << std::endl;
        Stop();
    }

    void Stop() {
        std::cout << "Stopping robot..." << std::endl;
        if (oaEnabled) {
            oaClient.Move(0, 0, 0);
            oaClient.UseRemoteCommandFromApi(false);
        }
        sportClient.StopMove();
    }

private:
    SportClient sportClient;
    ObstaclesAvoidClient oaClient;
    ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> stateSubscriber;
    unitree_go::msg::dds_::SportModeState_ state;
    RealSenseCamera camera;

    double maxRadius;
    size_t waypointIndex;
    double targetX, targetY;
    bool oaEnabled;
};

void PrintUsage(const char* program) {
    std::cout << "Usage: " << program << " [options]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --interface <name>    Network interface (default: enP8p1s0)" << std::endl;
    std::cout << "  --duration <seconds>  Exploration duration (default: 60)" << std::endl;
    std::cout << "  --area <size>         Area size: small, medium, large (default: medium)" << std::endl;
    std::cout << "  --no-oa               Disable obstacle avoidance" << std::endl;
    std::cout << "  --camera              Enable RealSense camera obstacle detection (C++ native)" << std::endl;
    std::cout << "  --help                Show this help" << std::endl;
}

int main(int argc, char** argv) {
    std::string interface = "enP8p1s0";
    int duration = 60;
    std::string area = "medium";
    bool useOA = true;
    bool useCamera = false;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--interface" && i + 1 < argc) {
            interface = argv[++i];
        } else if (arg == "--duration" && i + 1 < argc) {
            duration = std::atoi(argv[++i]);
        } else if (arg == "--area" && i + 1 < argc) {
            area = argv[++i];
        } else if (arg == "--no-oa") {
            useOA = false;
        } else if (arg == "--camera") {
            useCamera = true;
        } else if (arg == "--help") {
            PrintUsage(argv[0]);
            return 0;
        }
    }

    signal(SIGINT, signalHandler);

    std::cout << "============================================================" << std::endl;
    std::cout << "Unitree Go2 SLAM Explorer - All C++ Native" << std::endl;
    std::cout << "============================================================" << std::endl;
    std::cout << "Interface: " << interface << std::endl;
    std::cout << "Duration: " << duration << "s" << std::endl;
    std::cout << "Area: " << area << std::endl;
    std::cout << "Obstacle avoidance: " << (useOA ? "yes" : "no") << std::endl;
    std::cout << "Camera: " << (useCamera ? "yes (C++ native)" : "no") << std::endl;
    std::cout << "============================================================" << std::endl;

    // Initialize ChannelFactory FIRST
    ChannelFactory::Instance()->Init(0, interface);

    try {
        SLAMExplorer explorer;
        sleep(1);
        explorer.Explore(duration, area, useCamera);

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        ChannelFactory::Instance()->Release();
        return 1;
    }

    ChannelFactory::Instance()->Release();
    return 0;
}
