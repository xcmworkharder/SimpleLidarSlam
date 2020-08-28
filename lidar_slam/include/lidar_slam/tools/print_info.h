//
// Created by xcmworkharder on 2020-08-27 下午4:56.
//

#ifndef LIDAR_SLAM_PRINT_INFO_H
#define LIDAR_SLAM_PRINT_INFO_H

#include <cmath>
#include <string>
#include <Eigen/Dense>
#include "pcl/common/eigen.h"

namespace lidar_slam {
    class PrintInfo {
    public:
        static void PrintPose(std::string head, Eigen::Matrix4f pose);
    };
}

#endif //LIDAR_SLAM_PRINT_INFO_H
