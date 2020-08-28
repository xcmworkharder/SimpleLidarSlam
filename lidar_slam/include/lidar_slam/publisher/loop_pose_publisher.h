//
// Created by xcmworkharder on 2020-08-27 下午4:52.
//

#ifndef LIDAR_SLAM_LOOP_POSE_PUBLISHER_H
#define LIDAR_SLAM_LOOP_POSE_PUBLISHER_H

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "lidar_slam/sensor_data/loop_pose.h"

namespace lidar_slam {
    class LoopPosePublisher {
    public:
        LoopPosePublisher(ros::NodeHandle& nh,
                          std::string topic_name,
                          std::string frame_id,
                          int buff_size);
        LoopPosePublisher() = default;

        void Publish(LoopPose& loop_pose);

        bool HasSubscribers();

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        std::string frame_id_ = "";
    };
}

#endif //LIDAR_SLAM_LOOP_POSE_PUBLISHER_H
