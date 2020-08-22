//
// Created by xcmworkharder on 2020-08-21 上午10:13.
//

#ifndef LIDAR_SLAM_POSE_DATA_H
#define LIDAR_SLAM_POSE_DATA_H

#include <Eigen/Dense>

namespace lidar_slam {
    class PoseData {
    public:
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        double time = 0.0;

    public:
        Eigen::Quaternionf GetQuaternion();
    };
}

#endif //LIDAR_SLAM_POSE_DATA_H
