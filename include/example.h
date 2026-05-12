/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#pragma once

#include "unitree_lidar_sdk.h"

using namespace unilidar_sdk2;

static const std::string PATH_POINTS_CSV = "../data/points.csv";
static const std::string PATH_IMU_CSV = "../data/imu.csv";

void exampleProcess(UnitreeLidarReader *lreader){
    std::ofstream pts_file_;
    std::ofstream imu_file_;
    
    // open points CSV and write header
    pts_file_.open(PATH_POINTS_CSV, std::ios::out | std::ios::trunc);
    if (!pts_file_) {
        std::cerr << "[Logger] Cannot open " << PATH_POINTS_CSV << std::endl;
    } else {
        pts_file_ << "seq,x,y,z,intensity,time_offset,ring\n";
        std::cout << "[Logger] Writing points to " << PATH_POINTS_CSV << std::endl;
    }

    // open imu CSV qnd zrite header
    imu_file_.open(PATH_IMU_CSV, std::ios::out | std::ios::trunc);
    if (!imu_file_) {
        std::cerr << "[Logger] Cannot open " << PATH_IMU_CSV << std::endl;
    } else {
        imu_file_ << "seq,qw,qx,qy,qz,ang_x,ang_y,ang_z,acc_x,acc_y,acc_z\n";
        std::cout << "[Logger] Writing IMU data to " << PATH_IMU_CSV << std::endl;
    }

    // Get lidar version
    std::string versionSDK;
    std::string versionHardware;
    std::string versionFirmware;
    while (!lreader->getVersionOfLidarFirmware(versionFirmware))
    {
        lreader->runParse();
    }
    lreader->getVersionOfLidarHardware(versionHardware);
    lreader->getVersionOfSDK(versionSDK);

    std::cout << "lidar hardware version = " << versionHardware << std::endl
              << "lidar firmware version = " << versionFirmware << std::endl
              << "lidar sdk version = " << versionSDK << std::endl;
    sleep(1);

    // Parse PointCloud and IMU data
    int result;
    LidarImuData imu;
    PointCloudUnitree cloud;
    while (true)
    {
        result = lreader->runParse();

        switch (result)
        {
        case LIDAR_IMU_DATA_PACKET_TYPE:

            if (lreader->getImuData(imu))
            {
                if (imu_file_.is_open()) {
                    imu_file_ << imu.info.seq << ","
                        << std::fixed << std::setprecision(6)
                        << imu.quaternion[0] << "," << imu.quaternion[1] << "," << imu.quaternion[2] << "," << imu.quaternion[3] << ","
                        << imu.angular_velocity[0] << "," << imu.angular_velocity[1] << "," << imu.angular_velocity[2] << ","
                        << imu.linear_acceleration[0] << "," << imu.linear_acceleration[1] << "," << imu.linear_acceleration[2] << "\n";
                }
            }

            break;

        case LIDAR_POINT_DATA_PACKET_TYPE:
            if (lreader->getPointCloud(cloud))
            {
                if (pts_file_.is_open()) {
                    for (size_t i = 0; i < cloud.points.size(); i++) {
                        pts_file_ << cloud.id << ","
                            << std::fixed << std::setprecision(4)
                            << cloud.points[i].x << "," << cloud.points[i].y << "," << cloud.points[i].z << ","
                            << cloud.points[i].intensity << "," << cloud.points[i].time << "," << cloud.points[i].ring << "\n";
                    }
                }
            }

            break;

        default:
            break;
        }

    }
    if (pts_file_.is_open()) pts_file_.close();
    if (imu_file_.is_open()) imu_file_.close();
}
