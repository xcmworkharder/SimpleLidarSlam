//
// Created by xcmworkharder on 2020-08-21 上午10:08.
//

#ifndef LIDAR_SLAM_KEY_FRAMES_PUBLISHER_H
#define LIDAR_SLAM_KEY_FRAMES_PUBLISHER_H

#include <string>
#include <deque>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "lidar_slam/sensor_data/key_frame.h"

namespace lidar_slam {
    class KeyFramesPublisher {
    public:
        KeyFramesPublisher(ros::NodeHandle& nh,
                           std::string topic_name,
                           std::string frame_id,
                           int buff_size);
        KeyFramesPublisher() = default;

        void Publish(const std::deque<KeyFrame>& key_frames);
        bool HasSubscribers();

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        std::string frame_id_ = "";
    };
}

#endif //LIDAR_SLAM_KEY_FRAMES_PUBLISHER_H
