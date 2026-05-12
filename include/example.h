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

    /*
    // Stop and start lidar again
    std::cout << "stop lidar rotation ..." << std::endl;
    lreader->stopLidarRotation();
    sleep(3);

    std::cout << "start lidar rotation ..." << std::endl;
    lreader->startLidarRotation();
    sleep(3);

    // Check lidar dirty percentange
    float dirtyPercentage;
    while(!lreader->getDirtyPercentage(dirtyPercentage)){
        lreader->runParse();
    }
    printf("dirty percentage = %f %%\n", dirtyPercentage);
    sleep(1);

    // Get time delay
    double timeDelay;
    while(!lreader->getTimeDelay(timeDelay)){
        lreader->runParse();
    }
    printf("time delay (second) = %f\n", timeDelay);
    sleep(1);
    */

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
                //printf("An IMU msg is parsed!\n");
                /*
                std::cout << std::setprecision(20) << "\tsystem stamp = " << getSystemTimeStamp() << std::endl;
                printf("\tseq = %d, stamp = %d.%d\n", imu.info.seq, imu.info.stamp.sec, imu.info.stamp.nsec);

                printf("\tquaternion (x, y, z, w) = [%.4f, %.4f, %.4f, %.4f]\n",
                       imu.quaternion[0],
                       imu.quaternion[1],
                       imu.quaternion[2],
                       imu.quaternion[3]);

                printf("\tangular_velocity (x, y, z) = [%.4f, %.4f, %.4f]\n",
                       imu.angular_velocity[0],
                       imu.angular_velocity[1],
                       imu.angular_velocity[2]);
                printf("\tlinear_acceleration (x, y, z) = [%.4f, %.4f, %.4f]\n",
                       imu.linear_acceleration[0],
                       imu.linear_acceleration[1],
                       imu.linear_acceleration[2]);
                */
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
                //printf("A Cloud msg is parsed! \n");
                /*
                printf("\tstamp = %f, id = %d\n", cloud.stamp, cloud.id);
                printf("\tcloud size  = %ld, ringNum = %d\n", cloud.points.size(), cloud.ringNum);
                printf("\tfirst 10 points (x,y,z,intensity,time,ring) = \n");
                for (int i = 0; i < 10; i++)
                {
                    printf("\t  (%f, %f, %f, %f, %f, %d)\n",
                           cloud.points[i].x,
                           cloud.points[i].y,
                           cloud.points[i].z,
                           cloud.points[i].intensity,
                           cloud.points[i].time,
                           cloud.points[i].ring);
                }
                printf("\t  ...\n");
                */
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
