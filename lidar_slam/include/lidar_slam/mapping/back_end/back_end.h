//
// Created by xcmworkharder on 2020-08-21 上午9:30.
//
#ifndef LIDAR_SLAM_BACK_END_H
#define LIDAR_SLAM_BACK_END_H

#include <string>
#include <deque>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include "lidar_slam/sensor_data/cloud_data.h"
#include "lidar_slam/sensor_data/pose_data.h"
#include "lidar_slam/sensor_data/key_frame.h"

namespace lidar_slam {
    class BackEnd {
    public:
        BackEnd();

        bool Update(const CloudData& cloud_data, const PoseData& laser_odom,
                    const PoseData& gnss_pose);
        void GetOptimizedKeyFrames(std::deque<KeyFrame>& key_frames_deque);
        bool HasNewKeyFrame();
        bool HasNewOptimized();
        void GetLatestKeyFrame(KeyFrame& key_frame);

    private:
        bool InitWithConfig();
        bool InitParam(const YAML::Node& config_node);
        bool InitDataPath(const YAML::Node& config_node);
        void ResetParam();
        bool SaveTrajectory(const PoseData& laser_odom, const PoseData& gnss_pose);
        bool MaybeNewKeyFrame(const CloudData& cloud_data, const PoseData& laser_odom);
        bool MaybeOptimized();

    private:
        std::string key_frames_path_ = "";
        std::string trajectory_path_ = "";

        std::ofstream ground_truth_ofs_;
        std::ofstream laser_odom_ofs_;

        float key_frame_distance_ = 2.0;
        int optimize_step_with_none_ = 100;
        int optimize_step_with_gnss_ = 100;
        int optimize_step_with_loop_ = 10;

        bool has_new_key_frame_ = false;
        bool has_new_optimized_ = false;

        Eigen::Matrix4f last_key_pose_ = Eigen::Matrix4f::Identity();
        KeyFrame latest_key_frame_;
        std::deque<KeyFrame> key_frames_deque_;
    };
}

#endif //LIDAR_SLAM_BACK_END_H