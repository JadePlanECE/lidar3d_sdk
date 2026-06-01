/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "example.h"

bool force3DMode(unilidar_sdk2::UnitreeLidarReader *lreader, int timeout = 30)
{
    auto start_time = std::chrono::steady_clock::now();
    auto last_time = start_time;

    // Init config
    lreader->setLidarWorkMode(0);
    lreader->startLidarRotation();

    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(timeout))
    {
        int packetType = lreader->runParse();

        if (packetType == LIDAR_POINT_DATA_PACKET_TYPE) // 3D data
        {
            return true;
        }
        
        auto now = std::chrono::steady_clock::now();
        if (now - last_time > std::chrono::milliseconds(200)) {
            lreader->setLidarWorkMode(0);
            lreader->startLidarRotation();
            last_time = now;
        }
    }
    return false;
};

int main(int argc, char *argv[])
{
    // Initialize
    UnitreeLidarReader *lreader = createUnitreeLidarReader();

    std::string lidar_ip = "192.168.1.62";
    std::string local_ip = "192.168.1.2";

    unsigned short lidar_port = 6101;
    unsigned short local_port = 6201;

    if (lreader->initializeUDP(lidar_port, lidar_ip, local_port, local_ip))
    {
        printf("[Error] Unilidar initialization failed. Exit here!\n");
        exit(-1);
    }
    printf("[Initialization] Unilidar initialization succeed\n");

    if (!force3DMode(lreader, 30))
    {
        printf("[Error] Could not guarantee 3D mode. Exit here!\n");
        lreader->stopLidarRotation();
        exit(-1);
    }
    printf("[Initialization] Unilidar in 3D mode.\n");

    // Process
    exampleProcess(lreader);
    
    return 0;
}