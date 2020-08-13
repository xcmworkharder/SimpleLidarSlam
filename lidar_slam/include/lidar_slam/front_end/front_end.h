#ifndef LIDAR_LOCALIZATION_FRONT_END_FRONT_END_H_
#define LIDAR_LOCALIZATION_FRONT_END_FRONT_END_H_

#include <deque>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include "lidar_slam/sensor_data/cloud_data.h"

namespace lidar_localization {
    class FrontEnd {
    public:
        class Frame {
        public:
            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
            CloudData cloud_data;
        };

    public:
        FrontEnd();

        Eigen::Matrix4f Update(const CloudData& cloud_data);
        bool SetInitPose(const Eigen::Matrix4f& init_pose);
        bool SetPredictPose(const Eigen::Matrix4f& predict_pose);

        bool GetNewLocalMap(CloudData::CLOUD_PTR& local_map_ptr);
        bool GetNewGlobalMap(CloudData::CLOUD_PTR& global_map_ptr);
        bool GetCurrentScan(CloudData::CLOUD_PTR& current_scan_ptr);

    private:
        void UpdateNewFrame(const Frame& new_key_frame);

    private:
        pcl::VoxelGrid<CloudData::POINT> cloud_filter_;
        pcl::VoxelGrid<CloudData::POINT> local_map_filter_;
        pcl::VoxelGrid<CloudData::POINT> display_filter_;
        pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr ndt_ptr_;

        std::deque<Frame> local_map_frames_;
        std::deque<Frame> global_map_frames_;

        bool has_new_local_map_ = false;
        bool has_new_global_map_ = false;

        CloudData::CLOUD_PTR local_map_ptr_;
        CloudData::CLOUD_PTR global_map_ptr_;
        CloudData::CLOUD_PTR result_cloud_ptr_;
        Frame current_frame_;

        Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f predict_pose_ = Eigen::Matrix4f::Identity();
    };
}

#endif