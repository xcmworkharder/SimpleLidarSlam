#include "lidar_slam/front_end/front_end.h"
#include <fstream>
#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"

#include "lidar_slam/global_defination/global_defination.h"

namespace lidar_slam {
    FrontEnd::FrontEnd() : local_map_ptr_(new CloudData::CLOUD()),
                           global_map_ptr_(new CloudData::CLOUD()),
                           result_cloud_ptr_(new CloudData::CLOUD()) {
        InitWithConfig();
    }

    bool FrontEnd::InitWithConfig() {
        std::string config_file_path = WORK_SPACE_PATH + "/config/front_end/config.yaml";
        YAML::Node config_node = YAML::LoadFile(config_file_path);

        InitDataPath(config_node);
        InitRegistration(registration_ptr_, config_node);
        InitFilter("local_map", local_map_filter_ptr_, config_node);
        InitFilter("frame", frame_filter_ptr_, config_node);
        InitFilter("display", display_filter_ptr_, config_node);

        return true;
    }

    bool FrontEnd::InitParam(const YAML::Node& config_node) {
        key_frame_distance_ = config_node["key_frame_distance"].as<float>();
        local_frame_num_ = config_node["local_frame_num"].as<int>();

        return true;
    }

    bool FrontEnd::InitDataPath(const YAML::Node& config_node) {
        data_path_ = config_node["data_path"].as<std::string>();
        if (data_path_ == "./") {
            data_path_ = WORK_SPACE_PATH;
        }
        data_path_ += "/slam_data";

        if (boost::filesystem::is_directory(data_path_)) {
            boost::filesystem::remove_all(data_path_);
        }
        boost::filesystem::create_directories(data_path_);
        if (!boost::filesystem::is_directory(data_path_)) {
            LOG(WARNING) << "文件夹 " << data_path_ << "未创建成功！";
            return false;
        } else {
            LOG(INFO) << "地图点云存放地址: " << data_path_;
        }

        std::string key_frame_path = data_path_ + "/key_frames";
        boost::filesystem::create_directories(data_path_ + "/key_frames");
        if (!boost::filesystem::is_directory(key_frame_path)) {
            LOG(WARNING) << "文件夹: " << key_frame_path << "未创建成功！";
            return false;
        } else {
            LOG(INFO) << "关键帧点云存放地址： " << key_frame_path << std::endl << std::endl;
        }

        return true;
    }

    bool FrontEnd::InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr,
                                    const YAML::Node& config_node) {
        std::string registration_method = config_node["registration_method"].as<std::string>();
        LOG(INFO) << "点云匹配方式为: " << registration_method;

        if (registration_method == "NDT") {
            registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
        } else {
            LOG(ERROR) << "没找到与 " << registration_method << " 相应的点云匹配方式!";
            return false;
        }

        return true;
    }

    bool FrontEnd::InitFilter(std::string filter_user,
                              std::shared_ptr<CloudFilterInterface>& filter_ptr,
                              const YAML::Node& config_node) {
        std::string filter_method = config_node[filter_user + "_filter"].as<std::string>();
        LOG(INFO) << filter_user << "选择的滤波方法为: " << filter_method;

        if (filter_method == "voxel_filter") {
            filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_method][filter_user]);
        } else {
            LOG(ERROR) << "没有为 " << filter_user << " 找到与 " << filter_method << " 相对应的滤波方法";
            return false;
        }

        return true;
    }

    bool FrontEnd::Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose) {
        current_frame_.cloud_data.time = cloud_data.time;
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr,
                                     *current_frame_.cloud_data.cloud_ptr,
                                     indices);

        CloudData::CLOUD_PTR filter_cloud_ptr(new CloudData::CLOUD());
        frame_filter_ptr_->Filter(current_frame_.cloud_data.cloud_ptr, filter_cloud_ptr);

        static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
        static Eigen::Matrix4f last_pose = init_pose_;
        static Eigen::Matrix4f predict_pose = init_pose_;
        static Eigen::Matrix4f last_key_frame_pose = init_pose_;

        /// 如果是第一帧，直接作为关键帧
        if (local_map_frames_.size() == 0) {
            current_frame_.pose = init_pose_;
            UpdateWithNewFrame(current_frame_);
            cloud_pose = current_frame_.pose;
            return true;
        }

        /// 如果不是第一帧，就正常匹配
        registration_ptr_->ScanMatch(filter_cloud_ptr, predict_pose,
                                     result_cloud_ptr_, current_frame_.pose);
        cloud_pose = current_frame_.pose;

        /// 更新相邻两帧之间的相对运动
        step_pose = last_pose.inverse() * current_frame_.pose;
        predict_pose = current_frame_.pose * step_pose;
        last_pose = current_frame_.pose;

        /// 匹配之后根据距离判断是否生成新的关键帧，如果需要则进行更新,曼哈顿距离
        if (fabs(last_key_frame_pose(0, 3) - current_frame_.pose(0, 3)) +
            fabs(last_key_frame_pose(1, 3) - current_frame_.pose(1, 3)) +
            fabs(last_key_frame_pose(2, 3) - current_frame_.pose(2, 3))
                                                    > key_frame_distance_) {
            UpdateWithNewFrame(current_frame_);
            last_key_frame_pose = current_frame_.pose;
        }

        return true;
    }

    bool FrontEnd::SetInitPose(const Eigen::Matrix4f& init_pose) {
        init_pose_ = init_pose;
        return true;
    }

    bool FrontEnd::UpdateWithNewFrame(const Frame& new_key_frame) {
        /// 把关键帧点云存储到硬盘中，节省内存
        std::string file_path = data_path_ + "/key_frames/key_frame_" +
                                std::to_string(global_map_frames_.size()) +
                                ".pcd";
        pcl::io::savePCDFileBinary(file_path,
                                   *new_key_frame.cloud_data.cloud_ptr);

        Frame key_frame = new_key_frame;
        key_frame.cloud_data.cloud_ptr.reset(
                new CloudData::CLOUD(*new_key_frame.cloud_data.cloud_ptr));
        CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());

        /// 更新局部地图
        local_map_frames_.push_back(key_frame);
        /// 维护一个20的窗口
        while (local_map_frames_.size() > static_cast<size_t>(local_frame_num_)) {
            local_map_frames_.pop_front();
        }
        local_map_ptr_.reset(new CloudData::CLOUD());
        for (size_t i = 0; i < local_map_frames_.size(); ++i) {
            pcl::transformPointCloud(*local_map_frames_[i].cloud_data.cloud_ptr,
                                     *transformed_cloud_ptr,
                                     local_map_frames_[i].pose);
            *local_map_ptr_ += *transformed_cloud_ptr;
        }
        has_new_local_map_ = true;

        /// 更新ndt匹配的目标点云
        if (local_map_frames_.size() < 10) {
            registration_ptr_->SetInputTarget(local_map_ptr_);
        } else {
            CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
            local_map_filter_ptr_->Filter(local_map_ptr_,
                                          filtered_local_map_ptr);
            registration_ptr_->SetInputTarget(filtered_local_map_ptr);
        }

        /// 保存所有关键帧位姿到容器中，先释放点云
        key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD());
        global_map_frames_.push_back(key_frame);

        return true;
    }

    bool FrontEnd::SaveMap() {
        global_map_ptr_.reset(new CloudData::CLOUD());

        std::string key_frame_path = "";
        CloudData::CLOUD_PTR key_frame_cloud_ptr(new CloudData::CLOUD());
        CloudData::CLOUD_PTR  transformed_cloud_ptr(new CloudData::CLOUD());

        for (size_t i = 0; i < global_map_frames_.size(); ++i) {
            key_frame_path = data_path_ + "/key_frames/key_frame_" + std::to_string(i) + ".pcd";
            pcl::io::loadPCDFile(key_frame_path, *key_frame_cloud_ptr);

            pcl::transformPointCloud(*key_frame_cloud_ptr,
                                     *transformed_cloud_ptr,
                                     global_map_frames_[i].pose);
            *global_map_ptr_ += *transformed_cloud_ptr;
        }

        std::string map_file_path = data_path_ + "/map.pcd";
        pcl::io::savePCDFileBinary(map_file_path, *global_map_ptr_);
        has_new_global_map_ = true;

        return true;
    }

    bool FrontEnd::GetNewLocalMap(CloudData::CLOUD_PTR& local_map_ptr) {
        if (has_new_local_map_) {
            display_filter_ptr_->Filter(local_map_ptr_, local_map_ptr);
            return true;
        }
        return false;
    }

    bool FrontEnd::GetNewGlobalMap(CloudData::CLOUD_PTR& global_map_ptr) {
        if (has_new_global_map_) {
            has_new_global_map_ = false;
            display_filter_ptr_->Filter(global_map_ptr_, global_map_ptr);
            global_map_ptr_.reset(new CloudData::CLOUD());
            return true;
        }
        return false;
    }

    bool FrontEnd::GetCurrentScan(CloudData::CLOUD_PTR& current_scan_ptr) {
        display_filter_ptr_->Filter(result_cloud_ptr_, current_scan_ptr);
        return true;
    }
} // end of namespace