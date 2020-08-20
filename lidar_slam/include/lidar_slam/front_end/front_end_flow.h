//
// Created by xcmworkharder on 2020-08-13 下午6:41.
//

#ifndef LIDAR_SLAM_FRONT_END_FLOW_H
#define LIDAR_SLAM_FRONT_END_FLOW_H

#include <ros/ros.h>

#include "lidar_slam/subscriber/cloud_subscriber.h"
#include "lidar_slam/subscriber/imu_subscriber.h"
#include "lidar_slam/subscriber/velocity_subscriber.h"
#include "lidar_slam/subscriber/gnss_subscriber.h"
#include "lidar_slam/tf_listener/tf_listener.h"
#include "lidar_slam/publisher/cloud_publisher.h"
#include "lidar_slam/publisher/odometry_publisher.h"
#include "lidar_slam/front_end/front_end.h"
#include "lidar_slam/models/scan_adjust/distortion_adjust.h"

namespace lidar_slam {
    class FrontEndFlow {
    public:
        FrontEndFlow(ros::NodeHandle& nh);

        bool Run();
        bool SaveMap();
        bool PublishGlobalMap();

    private:
        bool ReadData();
        bool InitCalibration();
        bool InitGNSS();
        bool HasData();
        bool ValidData();
        bool UpdateGNSSOdometry();
        bool UpdateLaserOdometry();
        bool PublishData();
        bool SaveTrajectory();

    private:
        std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
        std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
        std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_;
        std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
        std::shared_ptr<TFListener> lidar_to_imu_ptr_;
        std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
        std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
        std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
        std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
        std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;
        std::shared_ptr<FrontEnd> front_end_ptr_;

        std::deque<CloudData> cloud_data_buff_;
        std::deque<IMUData> imu_data_buff_;
        std::deque<VelocityData> velocity_data_buff_;
        std::deque<GNSSData> gnss_data_buff_;
        Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();
        CloudData current_cloud_data_;
        IMUData current_imu_data_;
        VelocityData current_velocity_data_;
        GNSSData current_gnss_data_;

        CloudData::CLOUD_PTR local_map_ptr_;
        CloudData::CLOUD_PTR global_map_ptr_;
        CloudData::CLOUD_PTR current_scan_ptr_;
        Eigen::Matrix4f gnss_odometry_ = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();

        std::shared_ptr<DistortionAdjust> distortion_adjust_ptr_;
    };
}

#endif //LIDAR_SLAM_FRONT_END_FLOW_H
