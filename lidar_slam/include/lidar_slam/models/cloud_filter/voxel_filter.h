//
// Created by xcmworkharder on 2020-08-13 下午5:27.
//

#ifndef LIDAR_SLAM_VOXEL_FILTER_H
#define LIDAR_SLAM_VOXEL_FILTER_H

#include <pcl/filters/voxel_grid.h>
#include "lidar_slam/models/cloud_filter/cloud_filter_interface.h"

namespace lidar_slam {
    class VoxelFilter : public CloudFilterInterface {
    public:
        VoxelFilter(const YAML::Node& node);
        VoxelFilter(float lear_size_x, float leaf_size_y, float leaf_size_z);

        bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr,
                    CloudData::CLOUD_PTR& filtered_cloud_ptr) override;

    private:
        bool SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z);

    private:
        pcl::VoxelGrid<CloudData::POINT> voxel_filter_;
    };
}

#endif //LIDAR_SLAM_VOXEL_FILTER_H
