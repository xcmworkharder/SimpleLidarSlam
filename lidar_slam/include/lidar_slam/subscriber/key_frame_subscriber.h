//
// Created by xcmworkharder on 2020-08-21 上午10:17.
//

#ifndef LIDAR_SLAM_KEY_FRAME_SUBSCRIBER_H
#define LIDAR_SLAM_KEY_FRAME_SUBSCRIBER_H

#include <deque>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "lidar_slam/sensor_data/key_frame.h"

namespace lidar_slam {
    class KeyFrameSubscriber {
    public:
        KeyFrameSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
        KeyFrameSubscriber() = default;
        void ParseData(std::deque<KeyFrame>& key_frame_buff);

    private:
        void msg_callback(const geometry_msgs::PoseStampedConstPtr& key_frame_msg_ptr);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;

        std::deque<KeyFrame> new_key_frame_;
    };
}

#endif //LIDAR_SLAM_KEY_FRAME_SUBSCRIBER_H
