/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#pragma once

#include "unitree_lidar_sdk.h"
#include <filesystem>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <atomic>
#include <thread>

using namespace unilidar_sdk2;
namespace fs = std::filesystem;

static const std::string PATH_POINTS_CSV = "../data/points.csv";
static const std::string PATH_IMU_CSV = "../data/imu.csv";
static const int DELTA = 100;

void backupExistingFile(const std::string& filepath) {
    if (fs::exists(filepath)) {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
        
        fs::path p(filepath);
        std::string new_filename = p.stem().string() + "_" + ss.str() + p.extension().string();
        fs::path new_filepath = p.parent_path() / new_filename;

        try {
            fs::rename(p, new_filepath);
            std::cout << "[Backup] Renamed existing file to: " << new_filepath << std::endl;
        } catch (const fs::filesystem_error& e) {
            std::cerr << "[Backup Error] Failed to rename file: " << e.what() << std::endl;
        }
    }
}

void exampleProcess(UnitreeLidarReader *lreader){
    std::ofstream pts_file_;
    std::ofstream imu_file_;
    
    // backup the old files before opening new ones
    backupExistingFile(PATH_POINTS_CSV);
    backupExistingFile(PATH_IMU_CSV);

    // open points CSV and write header
    pts_file_.open(PATH_POINTS_CSV, std::ios::out | std::ios::trunc);
    if (!pts_file_) {
        std::cerr << "[Logger] Cannot open " << PATH_POINTS_CSV << std::endl;
    } else {
        pts_file_ << "id,time,x,y,z,intensity\n";
        pts_file_.flush();
        std::cout << "[Logger] Writing points to " << PATH_POINTS_CSV << std::endl;
    }

    // open imu CSV qnd write header
    imu_file_.open(PATH_IMU_CSV, std::ios::out | std::ios::trunc);
    if (!imu_file_) {
        std::cerr << "[Logger] Cannot open " << PATH_IMU_CSV << std::endl;
    } else {
        imu_file_ << "seq,time_sec,time_nsec,qw,qx,qy,qz,ang_x,ang_y,ang_z,acc_x,acc_y,acc_z\n";
        imu_file_.flush();
        std::cout << "[Logger] Writing IMU data to " << PATH_IMU_CSV << std::endl;
    }

    std::atomic<bool> stop_requested(false);

    // thread waiting for ENTER
    std::thread input_thread([&stop_requested]() {
        std::cout << "Press ENTER to stop...\n";
        std::cin.get();
        stop_requested = true;
    });

    // Parse PointCloud and IMU data
    int result;
    LidarImuData imu;
    PointCloudUnitree cloud;
    
    std::cout << "[Process] Lidar is going to start.." << std::endl;
    
    auto start = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed;
    int count = 0;
    int points_3d = 0;
    int points_2d = 0;
    int imu_info = 0;
    int invalide_message = 0;
    int others = 0;
    /*double sum_time = 0.0;
    int counter_time = 0;
    int sum_points = 0;
    int counter_points = 0;*/

    while (!stop_requested)
    {
        result = lreader->runParse();

        switch (result)
        {
        case LIDAR_IMU_DATA_PACKET_TYPE:
        {
            if (lreader->getImuData(imu))
            {
                if (imu_file_.is_open()) {
                    imu_file_ << imu.info.seq << ","
                        << imu.info.stamp.sec << "," << imu.info.stamp.nsec << ","
                        << std::fixed << std::setprecision(6)
                        << imu.quaternion[0] << "," << imu.quaternion[1] << "," << imu.quaternion[2] << "," << imu.quaternion[3] << ","
                        << imu.angular_velocity[0] << "," << imu.angular_velocity[1] << "," << imu.angular_velocity[2] << ","
                        << imu.linear_acceleration[0] << "," << imu.linear_acceleration[1] << "," << imu.linear_acceleration[2] << "\n";
                    imu_file_.flush();
                    imu_info += 1;
                }
            }
            break;
        }
        case LIDAR_POINT_DATA_PACKET_TYPE:
        {
            if (lreader->getPointCloud(cloud))
            {
                if (pts_file_.is_open()) {
                    for (size_t i = 0; i < cloud.points.size(); i++) {
                        pts_file_ << cloud.id << "," << cloud.stamp << ","
                            << std::fixed << std::setprecision(4)
                            << cloud.points[i].x << "," << cloud.points[i].y << "," << cloud.points[i].z << ","
                            << cloud.points[i].intensity << "\n";
                    }
                    pts_file_.flush();
                    points_3d += cloud.points.size();
                }
                /*elapsed = std::chrono::steady_clock::now() - start;
                auto elapsed_ms = elapsed.count() * 1000;
                std::cout << "[Process] Time to get data: " << elapsed_ms << " ms for " << cloud.points.size() << " points" << std::endl;
                sum_points += cloud.points.size();
                counter_points++;
                if (elapsed_ms < 1000)
                {
                    sum_time += elapsed_ms;
                    counter_time++;
                }
                start = std::chrono::steady_clock::now();*/
            }
            break;
        }
        case LIDAR_2D_POINT_DATA_PACKET_TYPE:
            points_2d += 1;
            std::cout << "[Warning] Received 2D data packet" << std::endl;
            break;

        case 0:
            // no valid message parsed
            // so we received invalide message
            invalide_message += 1;
            break;

        default:
            others += 1;
            //std::cout << result << std::endl;
            break;
        }

        elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed.count() * 1000 > DELTA)
        {
            count += 1;
        }
    }

    std::cout << "[Process] Stopping cleanly..." << std::endl;

    if (input_thread.joinable()) {
        input_thread.join();
    }
    
    if (pts_file_.is_open()) pts_file_.close();
    if (imu_file_.is_open()) imu_file_.close();

    std::cout << "[Result] For a delta of " << DELTA << ":" << std::endl;
    std::cout << "[Result] IMU info          - Sum: " << imu_info <<
     "\n                           - Mean: " << imu_info / static_cast<double>(count) * 100 << std::endl;
    std::cout << "[Result] 3D points         - Sum: " << points_3d <<
     "\n                           - Mean: " << points_3d / static_cast<double>(count) * 100 << std::endl;
    std::cout << "[Result] 2D points         - Sum: " << points_2d <<
     "\n                           - Mean: " << points_2d / static_cast<double>(count) * 100 << std::endl;
    std::cout << "[Result] Invalide messages - Sum: " <<
     invalide_message << "\n                           - Mean: " << invalide_message / static_cast<double>(count) * 100 << std::endl;
    std::cout << "[Result] Other messages    - Sum: " << others <<
     "\n                           - Mean: " << others / static_cast<double>(count) * 100 << std::endl;
    
    //std::cout << "[Result] Mean time getting data: " << sum_time / counter_time << " ms" << std::endl;
    //std::cout << "[Result] Mean points: " << sum_points / counter_points << " points" << std::endl;

    lreader->stopLidarRotation();
}
