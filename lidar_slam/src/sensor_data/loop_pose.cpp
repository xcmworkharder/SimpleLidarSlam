//
// Created by xcmworkharder on 2020-08-27 下午6:05.
//
#include "lidar_slam/sensor_data/loop_pose.h"

namespace lidar_slam {
    Eigen::Quaternionf LoopPose::GetQuaternion() {
        Eigen::Quaternionf q;
        q = pose.block<3, 3>(0, 0);

        return q;
    }
}
