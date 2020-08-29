//
// Created by xcmworkharder on 2020-08-28 下午3:56.
//

#ifndef LIDAR_SLAM_TF_BROADCASTER_H
#define LIDAR_SLAM_TF_BROADCASTER_H

#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>

namespace lidar_slam {
    class TFBroadCaster {
    public:
        TFBroadCaster(std::string frame_id, std::string child_frame_id);
        TFBroadCaster() = default;
        void SendTransform(Eigen::Matrix4f pose, double time);
    protected:
        tf::StampedTransform transform_;
        tf::TransformBroadcaster broadcaster_;
    };
}

#endif //LIDAR_SLAM_TF_BROADCASTER_H
