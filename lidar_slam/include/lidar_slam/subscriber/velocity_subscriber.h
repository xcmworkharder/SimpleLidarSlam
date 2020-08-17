//
// Created by xcmworkharder on 2020-08-15 上午8:28.
//
#ifndef LIDAR_SLAM_VELOCITY_SUBSCRIBER_H
#define LIDAR_SLAM_VELOCITY_SUBSCRIBER_H

#include <deque>
#include <ros/ros.h>
#include "geometry_msgs/TwistStamped.h"
#include "lidar_slam/sensor_data/velocity_data.h"

namespace lidar_slam {
    class VelocitySubscriber {
    public:
        VelocitySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
        VelocitySubscriber() = default;
        void ParseData(std::deque<VelocityData>& deque_velocity_data);

    private:
        void msg_callback(const geometry_msgs::TwistStampedConstPtr& twist_msg_ptr);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;

        std::deque<VelocityData> new_velocity_data_;
    };
}

#endif //LIDAR_SLAM_VELOCITY_SUBSCRIBER_H
