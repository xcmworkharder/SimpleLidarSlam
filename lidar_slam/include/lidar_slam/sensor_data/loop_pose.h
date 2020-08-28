//
// Created by xcmworkharder on 2020-08-27 下午4:53.
//

#ifndef LIDAR_SLAM_LOOP_POSE_H
#define LIDAR_SLAM_LOOP_POSE_H

#include <Eigen/Dense>

namespace lidar_slam {
    class LoopPose {
    public:
        double time = 0.0;
        unsigned int index0 = 0;
        unsigned int index1 = 0;
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

    public:
        Eigen::Quaternionf GetQuaternion();
    };
}

#endif //LIDAR_SLAM_LOOP_POSE_H
