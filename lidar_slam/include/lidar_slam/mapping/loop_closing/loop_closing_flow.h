//
// Created by xcmworkharder on 2020-08-27 下午4:43.
//

#ifndef LIDAR_SLAM_LOOP_CLOSING_FLOW_H
#define LIDAR_SLAM_LOOP_CLOSING_FLOW_H

#include <deque>
#include <ros/ros.h>
// subscriber
#include "lidar_slam/subscriber/key_frame_subscriber.h"
// publisher
#include "lidar_slam/publisher/loop_pose_publisher.h"
// loop closing
#include "lidar_slam/mapping/loop_closing/loop_closing.h"

namespace lidar_slam {
    class LoopClosingFlow {
    public:
        LoopClosingFlow(ros::NodeHandle& nh);

        bool Run();

    private:
        bool ReadData();
        bool HasData();
        bool ValidData();
        bool PublishData();

    private:
        // subscriber
        std::shared_ptr<KeyFrameSubscriber> key_frame_sub_ptr_;
        std::shared_ptr<KeyFrameSubscriber> key_gnss_sub_ptr_;
        // publisher
        std::shared_ptr<LoopPosePublisher> loop_pose_pub_ptr_;
        // loop closing
        std::shared_ptr<LoopClosing> loop_closing_ptr_;

        std::deque<KeyFrame> key_frame_buff_;
        std::deque<KeyFrame> key_gnss_buff_;

        KeyFrame current_key_frame_;
        KeyFrame current_key_gnss_;
    };
}

#endif //LIDAR_SLAM_LOOP_CLOSING_FLOW_H
