//
// Created by xcmworkharder on 2020-08-27 下午4:47.
//

#ifndef LIDAR_SLAM_NO_FILTER_H
#define LIDAR_SLAM_NO_FILTER_H

#include "lidar_slam/models/cloud_filter/cloud_filter_interface.h"

namespace lidar_slam {
    class NoFilter: public CloudFilterInterface {
    public:
        NoFilter();

        bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;
    };
}

#endif //LIDAR_SLAM_NO_FILTER_H
