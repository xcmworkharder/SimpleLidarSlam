//
// Created by xcmworkharder on 2020-08-13 下午5:28.
//

#ifndef LIDAR_SLAM_REGISTRATION_INTERFACE_H
#define LIDAR_SLAM_REGISTRATION_INTERFACE_H

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include "lidar_slam/sensor_data/cloud_data.h"

namespace lidar_slam {
    class RegistrationInterface {
    public:
    public:
        virtual ~RegistrationInterface() = default;

        virtual bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) = 0;
        virtual bool ScanMatch(const CloudData::CLOUD_PTR& input_source,
                               const Eigen::Matrix4f& predict_pose,
                               CloudData::CLOUD_PTR& result_cloud_ptr,
                               Eigen::Matrix4f& result_pose) = 0;
    };
}

#endif //LIDAR_SLAM_REGISTRATION_INTERFACE_H
