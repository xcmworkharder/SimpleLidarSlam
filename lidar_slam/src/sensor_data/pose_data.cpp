//
// Created by xcmworkharder on 2020-08-21 上午11:56.
//
#include "lidar_slam/sensor_data/pose_data.h"

namespace lidar_slam {
    Eigen::Quaternionf PoseData::GetQuaternion() {
        Eigen::Quaternionf q;
        q = pose.block<3, 3>(0, 0);

        return q;
    }
}
