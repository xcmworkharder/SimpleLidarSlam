//
// Created by xcmworkharder on 2020-08-21 上午10:07.
//

#ifndef LIDAR_SLAM_KEY_FRAME_PUBLISHER_H
#define LIDAR_SLAM_KEY_FRAME_PUBLISHER_H

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "lidar_slam/sensor_data/key_frame.h"

namespace lidar_slam {
    class KeyFramePublisher {
    public:
        KeyFramePublisher(ros::NodeHandle& nh,
                          std::string topic_name,
                          std::string frame_id,
                          int buff_size);
        KeyFramePublisher() = default;
        void Publish(KeyFrame& key_frame);
        bool HasSubscribers();

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        std::string frame_id_ = "";
    };
}

#endif //LIDAR_SLAM_KEY_FRAME_PUBLISHER_H
