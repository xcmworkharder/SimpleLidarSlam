//
// Created by xcmworkharder on 2020-08-27 下午4:42.
//

#ifndef LIDAR_SLAM_LOOP_CLOSING_H
#define LIDAR_SLAM_LOOP_CLOSING_H

#include <deque>
#include <Eigen/Dense>
#include <pcl/registration/ndt.h>
#include <yaml-cpp/yaml.h>

#include "lidar_slam/sensor_data/key_frame.h"
#include "lidar_slam/sensor_data/loop_pose.h"
#include "lidar_slam/models/registration/registration_interface.h"
#include "lidar_slam/models/cloud_filter/cloud_filter_interface.h"

namespace lidar_slam {
    class LoopClosing {
    public:
        LoopClosing();

        bool Update(const KeyFrame key_frame, const KeyFrame key_gnss);

        bool HasNewLoopPose();
        LoopPose& GetCurrentLoopPose();

    private:
        bool InitWithConfig();
        bool InitParam(const YAML::Node& config_node);
        bool InitDataPath(const YAML::Node& config_node);
        bool InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node);
        bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node);

        bool DetectNearestKeyFrame(int& key_frame_index);
        bool CloudRegistration(int key_frame_index);
        bool JointMap(int key_frame_index, CloudData::CLOUD_PTR& map_cloud_ptr, Eigen::Matrix4f& map_pose);
        bool JointScan(CloudData::CLOUD_PTR& scan_cloud_ptr, Eigen::Matrix4f& scan_pose);
        bool Registration(CloudData::CLOUD_PTR& map_cloud_ptr,
                          CloudData::CLOUD_PTR& scan_cloud_ptr,
                          Eigen::Matrix4f& scan_pose,
                          Eigen::Matrix4f& result_pose);

    private:
        std::string key_frames_path_ = "";
        int extend_frame_num_ = 3;
        int loop_step_ = 10;
        int diff_num_ = 100;
        float detect_area_ = 10.0;
        float fitness_score_limit_ = 2.0;

        std::shared_ptr<CloudFilterInterface> scan_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> map_filter_ptr_;
        std::shared_ptr<RegistrationInterface> registration_ptr_;

        std::deque<KeyFrame> all_key_frames_;
        std::deque<KeyFrame> all_key_gnss_;

        LoopPose current_loop_pose_;
        bool has_new_loop_pose_ = false;
    };
}

#endif //LIDAR_SLAM_LOOP_CLOSING_H
