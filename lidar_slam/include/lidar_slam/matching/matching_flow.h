//
// Created by xcmworkharder on 2020-08-28 下午3:50.
//

#ifndef LIDAR_SLAM_MATCHING_FLOW_H
#define LIDAR_SLAM_MATCHING_FLOW_H

#include <ros/ros.h>
// subscriber
#include "lidar_slam/subscriber/cloud_subscriber.h"
#include "lidar_slam/subscriber/odometry_subscriber.h"
// publisher
#include "lidar_slam/publisher/cloud_publisher.h"
#include "lidar_slam/publisher/odometry_publisher.h"
#include "lidar_slam/publisher/tf_broadcaster.h"
// matching
#include "lidar_slam/matching/matching.h"

namespace lidar_slam {
    class MatchingFlow {
    public:
        MatchingFlow(ros::NodeHandle& nh);

        bool Run();

    private:
        bool ReadData();
        bool HasData();
        bool ValidData();
        bool UpdateMatching();
        bool PublishData();

    private:
        // subscriber
        std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
        std::shared_ptr<OdometrySubscriber> gnss_sub_ptr_;
        // publisher
        std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
        std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
        std::shared_ptr<CloudPublisher> current_scan_pub_ptr_;
        std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
        std::shared_ptr<TFBroadCaster> laser_tf_pub_ptr_;
        // matching
        std::shared_ptr<Matching> matching_ptr_;

        std::deque<CloudData> cloud_data_buff_;
        std::deque<PoseData> gnss_data_buff_;

        CloudData current_cloud_data_;
        PoseData current_gnss_data_;

        Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();
    };
}

#endif //LIDAR_SLAM_MATCHING_FLOW_H
