//
// Created by xcmworkharder on 2020-08-28 下午3:50.
//

#ifndef LIDAR_SLAM_MATCHING_H
#define LIDAR_SLAM_MATCHING_H

#include <deque>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "lidar_slam/sensor_data/cloud_data.h"
#include "lidar_slam/sensor_data/pose_data.h"

#include "lidar_slam/models/registration/registration_interface.h"
#include "lidar_slam/models/cloud_filter/cloud_filter_interface.h"
#include "lidar_slam/models/cloud_filter/box_filter.h"

namespace lidar_slam {
    class Matching {
    public:
        Matching();

        bool Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose);
        bool SetGNSSPose(const Eigen::Matrix4f& init_pose);

        void GetGlobalMap(CloudData::CLOUD_PTR& global_map);
        CloudData::CLOUD_PTR& GetLocalMap();
        CloudData::CLOUD_PTR& GetCurrentScan();
        bool HasInited();
        bool HasNewGlobalMap();
        bool HasNewLocalMap();

    private:
        bool InitWithConfig();
        bool InitDataPath(const YAML::Node& config_node);
        bool InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node);
        bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node);
        bool InitBoxFilter(const YAML::Node& config_node);

        bool SetInitPose(const Eigen::Matrix4f& init_pose);
        bool InitGlobalMap();
        bool ResetLocalMap(float x, float y, float z);

    private:
        std::string map_path_ = "";

        std::shared_ptr<BoxFilter> box_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;
        std::shared_ptr<RegistrationInterface> registration_ptr_;

        CloudData::CLOUD_PTR local_map_ptr_;
        CloudData::CLOUD_PTR global_map_ptr_;
        CloudData::CLOUD_PTR current_scan_ptr_;
        Eigen::Matrix4f current_pose_ = Eigen::Matrix4f::Identity();

        Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f current_gnss_pose_ = Eigen::Matrix4f::Identity();
        bool has_inited_ = false;
        bool has_new_global_map_ = false;
        bool has_new_local_map_ = false;
    };
}

#endif //LIDAR_SLAM_MATCHING_H
