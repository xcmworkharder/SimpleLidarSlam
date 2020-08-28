//
// Created by xcmworkharder on 2020-08-27 下午4:55.
//

#ifndef LIDAR_SLAM_LOOP_POSE_SUBSCRIBER_H
#define LIDAR_SLAM_LOOP_POSE_SUBSCRIBER_H

#include <deque>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "lidar_slam/sensor_data/loop_pose.h"

namespace lidar_slam {
    class LoopPoseSubscriber {
    public:
        LoopPoseSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
        LoopPoseSubscriber() = default;
        void ParseData(std::deque<LoopPose>& loop_pose_buff);

    private:
        void msg_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& loop_pose_msg_ptr);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;

        std::deque<LoopPose> new_loop_pose_;
    };
}

#endif //LIDAR_SLAM_LOOP_POSE_SUBSCRIBER_H
