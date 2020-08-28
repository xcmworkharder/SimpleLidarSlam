#include "lidar_slam/mapping/front_end/front_end_flow.h"
#include "glog/logging.h"
#include "lidar_slam/global_defination/global_defination.h"

namespace lidar_slam {
    FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh, std::string cloud_topic, std::string odom_topic) {
        cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, cloud_topic, 100000);
        laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, odom_topic, "/map", "/lidar", 100);

        front_end_ptr_ = std::make_shared<FrontEnd>();
    }

    bool FrontEndFlow::Run() {
        if (!ReadData())
            return false;

        while(HasData()) {
            if (!ValidData())
                continue;

            if (UpdateLaserOdometry()) {
                PublishData();
            }
        }

        return true;
    }

    bool FrontEndFlow::ReadData() {
        cloud_sub_ptr_->ParseData(cloud_data_buff_);
        return true;
    }

    bool FrontEndFlow::HasData() {
        return cloud_data_buff_.size() > 0;
    }

    bool FrontEndFlow::ValidData() {
        current_cloud_data_ = cloud_data_buff_.front();
        cloud_data_buff_.pop_front();

        return true;
    }

    bool FrontEndFlow::UpdateLaserOdometry() {
        static bool odometry_inited = false;
        if (!odometry_inited) {
            odometry_inited = true;
            front_end_ptr_->SetInitPose(Eigen::Matrix4f::Identity());
            return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
        }

        return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
    }

    bool FrontEndFlow::PublishData() {
        laser_odom_pub_ptr_->Publish(laser_odometry_, current_cloud_data_.time);

        return true;
    }
}