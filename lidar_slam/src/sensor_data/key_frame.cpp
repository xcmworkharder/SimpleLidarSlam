//
// Created by xcmworkharder on 2020-08-21 上午11:55.
//
#include "lidar_slam/sensor_data/key_frame.h"

namespace lidar_slam {
    Eigen::Quaternionf KeyFrame::GetQuaternion() {
        Eigen::Quaternionf q;
        q = pose.block<3, 3>(0, 0);

        return q;
    }
}
