/**
 * Description: 发布tf的类
 */
#include "lidar_slam/publisher/tf_broadcaster.h"

namespace lidar_slam {
    TFBroadCaster::TFBroadCaster(std::string frame_id, std::string child_frame_id) {
        transform_.frame_id_ = frame_id;
        transform_.child_frame_id_ = child_frame_id;
    }

    void TFBroadCaster::SendTransform(Eigen::Matrix4f pose, double time) {
        Eigen::Quaternionf q(pose.block<3, 3>(0, 0));
        ros::Time ros_time((float)time);
        transform_.stamp_ = ros_time;
        transform_.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
        transform_.setOrigin(tf::Vector3(pose(0, 3), pose(1, 3), pose(2, 3)));
        broadcaster_.sendTransform(transform_);
    }
}
