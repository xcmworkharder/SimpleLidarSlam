//
// Created by xcmworkharder on 2020-08-21 上午9:12.
//

#ifndef LIDAR_SLAM_DATA_PREPROCESS_H
#define LIDAR_SLAM_DATA_PREPROCESS_H

#include <ros/ros.h>
// subscriber
#include "lidar_slam/subscriber/cloud_subscriber.h"
#include "lidar_slam/subscriber/imu_subscriber.h"
#include "lidar_slam/subscriber/velocity_subscriber.h"
#include "lidar_slam/subscriber/gnss_subscriber.h"
#include "lidar_slam/tf_listener/tf_listener.h"
// publisher
#include "lidar_slam/publisher/cloud_publisher.h"
#include "lidar_slam/publisher/odometry_publisher.h"
// models
#include "lidar_slam/models/scan_adjust/distortion_adjust.h"

namespace lidar_slam {
    class DataPreprocessFlow {
    public:
        //DataPreprocessFlow(ros::NodeHandle& nh);
        DataPreprocessFlow(ros::NodeHandle& nh, std::string cloud_topic);
        bool Run();

    private:
        bool ReadData();
        bool InitCalibration();
        bool InitGNSS();
        bool HasData();
        bool ValidData();
        bool TransformData();
        bool PublishData();

    private:
        // subscriber
        std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
        std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
        std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_;
        std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
        std::shared_ptr<TFListener> lidar_to_imu_ptr_;

        // publisher
        std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
        std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;

        // models
        std::shared_ptr<DistortionAdjust> distortion_adjust_ptr_;

        Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();

        std::deque<CloudData> cloud_data_buff_;
        std::deque<IMUData> imu_data_buff_;
        std::deque<VelocityData> velocity_data_buff_;
        std::deque<GNSSData> gnss_data_buff_;

        CloudData current_cloud_data_;
        IMUData current_imu_data_;
        VelocityData current_velocity_data_;
        GNSSData current_gnss_data_;

        Eigen::Matrix4f gnss_pose_ = Eigen::Matrix4f::Identity();
    };
}

#endif //LIDAR_SLAM_DATA_PREPROCESS_H
