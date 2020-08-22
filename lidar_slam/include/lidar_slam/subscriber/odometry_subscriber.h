//
// Created by xcmworkharder on 2020-08-21 上午10:22.
//

#ifndef LIDAR_SLAM_ODOMETRY_SUBSCRIBER_H
#define LIDAR_SLAM_ODOMETRY_SUBSCRIBER_H

#include <deque>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "lidar_slam/sensor_data/pose_data.h"

namespace lidar_slam {
    class OdometrySubscriber {
    public:
        OdometrySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
        OdometrySubscriber() = default;
        void ParseData(std::deque<PoseData>& deque_pose_data);

    private:
        void msg_callback(const nav_msgs::OdometryConstPtr& odom_msg_ptr);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;

        std::deque<PoseData> new_pose_data_;
    };
}

#endif //LIDAR_SLAM_ODOMETRY_SUBSCRIBER_H
