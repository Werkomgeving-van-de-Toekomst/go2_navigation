#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp>
#include <iostream>
#include <unistd.h>

using namespace unitree::robot;
using namespace unitree::robot::go2;

int main(int argc, char** argv) {
    std::string interface = "enP8p1s0";
    if (argc >= 2) {
        interface = argv[1];
    }

    ChannelFactory::Instance()->Init(0, interface);
    
    SportClient sportClient;
    sportClient.SetTimeout(10.0f);
    sportClient.Init();

    ObstaclesAvoidClient oaClient;
    oaClient.SetTimeout(10.0f);
    oaClient.Init();

    // First disable obstacle avoidance
    std::cout << "Disabling obstacle avoidance..." << std::endl;
    oaClient.UseRemoteCommandFromApi(false);
    oaClient.SwitchSet(false);
    usleep(100000);

    // Stop any movement
    std::cout << "Stopping movement..." << std::endl;
    sportClient.StopMove();
    usleep(100000);

    // Put robot in damping mode (relaxed joints)
    std::cout << "Putting robot in damping mode..." << std::endl;
    sportClient.Damp();
    
    std::cout << "Robot is now in damping mode (relaxed joints)." << std::endl;
    
    ChannelFactory::Instance()->Release();
    return 0;
}
