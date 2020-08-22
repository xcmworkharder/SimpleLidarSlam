//
// Created by xcmworkharder on 2020-08-21 上午10:12.
//

#ifndef LIDAR_SLAM_KEY_FRAME_H
#define LIDAR_SLAM_KEY_FRAME_H

#include <Eigen/Dense>

namespace lidar_slam {
    class KeyFrame {
    public:
        double time = 0.0;
        unsigned int index = 0;
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

    public:
        Eigen::Quaternionf GetQuaternion();
    };
}

#endif //LIDAR_SLAM_KEY_FRAME_H
