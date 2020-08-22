//
// Created by xcmworkharder on 2020-08-21 上午9:40.
//

#ifndef LIDAR_SLAM_BACK_END_FLOW_H
#define LIDAR_SLAM_BACK_END_FLOW_H

#include <ros/ros.h>

#include "lidar_slam/subscriber/cloud_subscriber.h"
#include "lidar_slam/subscriber/odometry_subscriber.h"
#include "lidar_slam/publisher/odometry_publisher.h"
#include "lidar_slam/publisher/key_frame_publisher.h"
#include "lidar_slam/publisher/key_frames_publisher.h"
#include "lidar_slam/mapping/back_end/back_end.h"

namespace lidar_slam {
    class BackEndFlow {
    public:
        BackEndFlow(ros::NodeHandle& nh);
        bool Run();

    private:
        bool ReadData();
        bool HasData();
        bool ValidData();
        bool UpdateBackEnd();
        bool SaveTrajectory();
        bool PublishData();

    private:
        std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
        std::shared_ptr<OdometrySubscriber> gnss_pose_sub_ptr_;
        std::shared_ptr<OdometrySubscriber> laser_odom_sub_ptr_;

        std::shared_ptr<OdometryPublisher> transformed_odom_pub_ptr_;
        std::shared_ptr<KeyFramePublisher> key_frame_pub_ptr_;
        std::shared_ptr<KeyFramesPublisher> key_frames_pub_ptr_;
        std::shared_ptr<BackEnd> back_end_ptr_;

        std::deque<CloudData> cloud_data_buff_;
        std::deque<PoseData> gnss_pose_data_buff_;
        std::deque<PoseData> laser_odom_data_buff_;

        PoseData current_gnss_pose_data_;
        PoseData current_laser_odom_data_;
        CloudData current_cloud_data_;
    };
}

#endif //LIDAR_SLAM_BACK_END_FLOW_H
