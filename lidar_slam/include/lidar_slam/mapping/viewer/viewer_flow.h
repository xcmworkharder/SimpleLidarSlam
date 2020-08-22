//
// Created by xcmworkharder on 2020-08-21 上午9:54.
//

#ifndef LIDAR_SLAM_VIEWER_FLOW_H
#define LIDAR_SLAM_VIEWER_FLOW_H

#include <deque>
#include <ros/ros.h>
// subscriber
#include "lidar_slam/subscriber/cloud_subscriber.h"
#include "lidar_slam/subscriber/odometry_subscriber.h"
#include "lidar_slam/subscriber/key_frame_subscriber.h"
#include "lidar_slam/subscriber/key_frames_subscriber.h"
// publisher
#include "lidar_slam/publisher/odometry_publisher.h"
#include "lidar_slam/publisher/cloud_publisher.h"
// viewer
#include "lidar_slam/mapping/viewer/viewer.h"

namespace lidar_slam {
    class ViewerFlow {
    public:
        ViewerFlow(ros::NodeHandle& nh);

        bool Run();
        bool SaveMap();

    private:
        bool ReadData();
        bool HasData();
        bool ValidData();
        bool UpdateViewer();
        bool PublishData();

    private:
        // subscriber
        std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
        std::shared_ptr<OdometrySubscriber> transformed_odom_sub_ptr_;
        std::shared_ptr<KeyFrameSubscriber> key_frame_sub_ptr_;
        std::shared_ptr<KeyFramesSubscriber> optimized_key_frames_sub_ptr_;
        // publisher
        std::shared_ptr<OdometryPublisher> optimized_odom_pub_ptr_;
        std::shared_ptr<CloudPublisher> current_scan_pub_ptr_;
        std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
        std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
        // viewer
        std::shared_ptr<Viewer> viewer_ptr_;

        std::deque<CloudData> cloud_data_buff_;
        std::deque<PoseData> transformed_odom_buff_;
        std::deque<KeyFrame> key_frame_buff_;
        std::deque<KeyFrame> optimized_key_frames_;
        std::deque<KeyFrame> all_key_frames_;

        CloudData current_cloud_data_;
        PoseData current_transformed_odom_;
    };
}

#endif //LIDAR_SLAM_VIEWER_FLOW_H
