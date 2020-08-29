//
// Created by xcmworkharder on 2020-08-28 下午3:53.
//

#ifndef LIDAR_SLAM_BOX_FILTER_H
#define LIDAR_SLAM_BOX_FILTER_H

#include <pcl/filters/crop_box.h>
#include "lidar_slam/models/cloud_filter/cloud_filter_interface.h"

namespace lidar_slam {
    class BoxFilter: public CloudFilterInterface {
    public:
        BoxFilter(YAML::Node node);
        BoxFilter() = default;

        bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;

        void SetSize(std::vector<float> size);
        void SetOrigin(std::vector<float> origin);
        std::vector<float> GetEdge();

    private:
        void CalculateEdge();

    private:
        pcl::CropBox<CloudData::POINT> pcl_box_filter_;

        std::vector<float> origin_;
        std::vector<float> size_;
        std::vector<float> edge_;
    };
}

#endif //LIDAR_SLAM_BOX_FILTER_H
