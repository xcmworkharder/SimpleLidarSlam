//
// Created by xcmworkharder on 2020-08-13 下午5:26.
//

#ifndef LIDAR_SLAM_CLOUD_FILTER_INTERFACE_H
#define LIDAR_SLAM_CLOUD_FILTER_INTERFACE_H

#include <yaml-cpp/yaml.h>
#include "lidar_slam/sensor_data/cloud_data.h"

namespace lidar_slam {
    class CloudFilterInterface {
    public:
        virtual ~CloudFilterInterface() = default;

        //virtual bool Filter(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) = 0;
        virtual bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) = 0;
    };
}

#endif //LIDAR_SLAM_CLOUD_FILTER_INTERFACE_H
