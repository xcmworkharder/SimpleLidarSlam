//
// Created by xcmworkharder on 2020-08-17 下午9:32.
//

#ifndef LIDAR_SLAM_DISTORTION_ADJUST_H
#define LIDAR_SLAM_DISTORTION_ADJUST_H

#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include "glog/logging.h"

#include "lidar_slam/sensor_data/velocity_data.h"
#include "lidar_slam/sensor_data/cloud_data.h"

namespace lidar_slam {
    class DistortionAdjust {
    public:
        void SetMotionInfo(float scan_period, const VelocityData& velocity_data);
        bool AdjustCloud(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& output_cloud_ptr);

    private:
        inline Eigen::Matrix3f UpdateMatrix(float real_time);

    private:
        float scan_period_;
        Eigen::Vector3f velocity_;
        Eigen::Vector3f angular_rate_;
    };
}

#endif //LIDAR_SLAM_DISTORTION_ADJUST_H
