//
// Created by xcmworkharder on 2020-08-13 下午8:41.
//
#include "lidar_slam/front_end/front_end_flow.h"
#include "glog/logging.h"
#include "lidar_slam/tools/file_manager.h"
#include "lidar_slam/global_defination/global_defination.h"

namespace lidar_slam {
    FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh) {
        cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
        imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
        velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, "/kitti/oxts/gps/vel", 1000000);
        gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
        lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh, "imu_link", "velo_link");
        //lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh, "velo_link", "imu_link");

        cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "current_scan", 100, "/map");
        local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "local_map", 100, "/map");
        global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "global_map", 100, "/map");
        laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "laser_odom", "map", "lidar", 100);
        gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "gnss", "map", "lidar", 100);

        front_end_ptr_ = std::make_shared<FrontEnd>();
        distortion_adjust_ptr_ = std::make_shared<DistortionAdjust>();

        local_map_ptr_.reset(new CloudData::CLOUD());
        global_map_ptr_.reset(new CloudData::CLOUD());
        current_scan_ptr_.reset(new CloudData::CLOUD());
    }

    bool FrontEndFlow::Run() {
        if (!ReadData()) {
            return false;
        }

        if (!InitCalibration())
            return false;

        if (!InitGNSS())
            return false;

        while (HasData()) {
            if (!ValidData())
                continue;
            UpdateGNSSOdometry();
            if (UpdateLaserOdometry()) {
                PublishData();
                SaveTrajectory();
            }
        }

        return true;
    }

    bool FrontEndFlow::ReadData() {
        cloud_sub_ptr_->ParseData(cloud_data_buff_);

        static std::deque<IMUData> unsynced_imu_;
        static std::deque<VelocityData> unsynced_velocity_;
        static std::deque<GNSSData> unsynced_gnss_;
        imu_sub_ptr_->ParseData(unsynced_imu_);
        velocity_sub_ptr_->ParseData(unsynced_velocity_);
        gnss_sub_ptr_->ParseData(unsynced_gnss_);

        if (cloud_data_buff_.size() == 0) {
            return false;
        }

        double cloud_time = cloud_data_buff_.front().time;
        bool valid_imu = IMUData::SyncData(unsynced_imu_, imu_data_buff_, cloud_time);
        bool valid_velocity = VelocityData::SynData(unsynced_velocity_, velocity_data_buff_, cloud_time);
        bool valid_gnss = GNSSData::SyncData(unsynced_gnss_, gnss_data_buff_, cloud_time);

        static bool sensor_inited = false;
        if (!sensor_inited) {
            if (!valid_imu || !valid_velocity || !valid_gnss) {
                cloud_data_buff_.pop_front();
                return false;
            }
            sensor_inited = true;
        }

        return true;
    }

    bool FrontEndFlow::InitCalibration() {
        static bool calibration_received = false;
        if (!calibration_received) {
            if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) {
                calibration_received = true;
                LOG(INFO) << lidar_to_imu_;
            }
        }

        return calibration_received;
    }

    bool FrontEndFlow::InitGNSS() {
        static bool gnss_inited = false;
        if (!gnss_inited) {
            GNSSData gnss_data = gnss_data_buff_.front();
            gnss_data.InitOriginPosition();
            gnss_inited = true;
        }

        return gnss_inited;
    }

    bool FrontEndFlow::HasData() {
        if (cloud_data_buff_.size() == 0)
            return false;
        if (imu_data_buff_.size() == 0)
            return false;
        if (velocity_data_buff_.size() == 0)
            return false;
        if (gnss_data_buff_.size() == 0)
            return false;

        return true;
    }

    bool FrontEndFlow::ValidData() {
        current_cloud_data_ = cloud_data_buff_.front();
        current_imu_data_ = imu_data_buff_.front();
        current_velocity_data_ = velocity_data_buff_.front();
        current_gnss_data_ = gnss_data_buff_.front();

        //double d_time = current_cloud_data_.time - current_imu_data_.time;
        double d_time = current_cloud_data_.time - current_velocity_data_.time;
        if (d_time < -0.05) {
            cloud_data_buff_.pop_front();
            return false;
        }

        if (d_time > 0.05) {
            imu_data_buff_.pop_front();
            velocity_data_buff_.pop_front();
            gnss_data_buff_.pop_front();
            return false;
        }

        cloud_data_buff_.pop_front();
        imu_data_buff_.pop_front();
        velocity_data_buff_.pop_front();
        gnss_data_buff_.pop_front();

        return true;
    }

    bool FrontEndFlow::UpdateGNSSOdometry() {
        gnss_odometry_ = Eigen::Matrix4f::Identity();

        current_gnss_data_.UpdateXYZ();
        gnss_odometry_(0, 3) = current_gnss_data_.local_E;
        gnss_odometry_(1, 3) = current_gnss_data_.local_N;
        gnss_odometry_(2, 3) = current_gnss_data_.local_U;
        gnss_odometry_.block<3, 3>(0, 0) = current_imu_data_.GetOrientationMatrix();
        gnss_odometry_ *= lidar_to_imu_;

        return true;
    }

    bool FrontEndFlow::UpdateLaserOdometry() {
        current_velocity_data_.TransformCoordinate(lidar_to_imu_);
        distortion_adjust_ptr_->SetMotionInfo(0.1, current_velocity_data_);
        distortion_adjust_ptr_->AdjustCloud(current_cloud_data_.cloud_ptr,
                                            current_cloud_data_.cloud_ptr);

        static bool front_end_pose_inited = false;
        if (!front_end_pose_inited) {
            front_end_pose_inited = true;
            front_end_ptr_->SetInitPose(gnss_odometry_);
            return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
        }

        laser_odometry_ = Eigen::Matrix4f::Identity();
        return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
    }

    bool FrontEndFlow::PublishData() {
        gnss_pub_ptr_->Publish(gnss_odometry_);
        laser_odom_pub_ptr_->Publish(laser_odometry_);

        front_end_ptr_->GetCurrentScan(current_scan_ptr_);
        cloud_pub_ptr_->Publish(current_scan_ptr_);

        if (front_end_ptr_->GetNewLocalMap(local_map_ptr_))
            local_map_pub_ptr_->Publish(local_map_ptr_);

        return true;
    }

    bool FrontEndFlow::SaveMap() {
        return front_end_ptr_->SaveMap();
    }

    bool FrontEndFlow::PublishGlobalMap() {
        if (front_end_ptr_->GetNewGlobalMap(global_map_ptr_)) {
            global_map_pub_ptr_->Publish(global_map_ptr_);
            global_map_ptr_.reset(new CloudData::CLOUD());
        }
        return true;
    }

    bool FrontEndFlow::SaveTrajectory() {
        static std::ofstream ground_truth, laser_odom;
        static bool is_file_created = false;

        if (!is_file_created) {
            if (!FileManager::CreateDirectory(WORK_SPACE_PATH + "/slam_data/trajectory")) {
                return false;
            }
            if (!FileManager::CreateFile(ground_truth,
                                         WORK_SPACE_PATH + "/slam_data/trajectory/ground_truth.txt")) {
                return false;
            }
            if (!FileManager::CreateFile(laser_odom,
                                         WORK_SPACE_PATH + "/slam_data/trajectory/laser_odom.txt")) {
                return false;
            }
            is_file_created = true;
        }

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 4; ++j) {
                ground_truth << gnss_odometry_(i, j);
                laser_odom << laser_odometry_(i, j);
                if (i == 2 && j == 3) {
                    ground_truth << std::endl;
                    laser_odom << std::endl;
                } else {
                    ground_truth << " ";
                    laser_odom << " ";
                }
            }
        }

        return true;
    }
}
