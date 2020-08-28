//
// Created by xcmworkharder on 2020-08-27 下午5:49.
//
#include "lidar_slam/models/cloud_filter/no_filter.h"
#include "glog/logging.h"

namespace lidar_slam {
    NoFilter::NoFilter() {
    }

    bool NoFilter::Filter(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) {
        filtered_cloud_ptr.reset(new CloudData::CLOUD(*input_cloud_ptr));
        return true;
    }
}
