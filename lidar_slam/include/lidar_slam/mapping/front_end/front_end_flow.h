//
// Created by xcmworkharder on 2020-08-13 下午6:41.
//

#ifndef LIDAR_SLAM_FRONT_END_FLOW_H
#define LIDAR_SLAM_FRONT_END_FLOW_H

#include <ros/ros.h>

#include "lidar_slam/subscriber/cloud_subscriber.h"
#include "lidar_slam/publisher/odometry_publisher.h"
#include "lidar_slam/mapping/front_end/front_end.h"

namespace lidar_slam {
    class FrontEndFlow {
    public:
        FrontEndFlow(ros::NodeHandle& nh);

        bool Run();

    private:
        bool ReadData();
        bool HasData();
        bool ValidData();
        bool UpdateLaserOdometry();
        bool PublishData();

    private:
        std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
        std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
        std::shared_ptr<FrontEnd> front_end_ptr_;

        std::deque<CloudData> cloud_data_buff_;

        CloudData current_cloud_data_;

        Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();
    };
}

#endif //LIDAR_SLAM_FRONT_END_FLOW_H
