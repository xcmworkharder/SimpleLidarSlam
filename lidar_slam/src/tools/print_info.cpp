//
// Created by xcmworkharder on 2020-08-27 下午6:17.
//
#include "lidar_slam/tools/print_info.h"
#include "glog/logging.h"

namespace lidar_slam {
    void PrintInfo::PrintPose(std::string head, Eigen::Matrix4f pose) {
        Eigen::Affine3f aff_pose;
        aff_pose.matrix() = pose;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(aff_pose, x, y, z, roll, pitch, yaw);
        std::cout << head
                  << x << "," << y << "," << z << ","
                  << roll * 180 / M_PI << "," << pitch * 180 / M_PI << "," << yaw * 180 / M_PI
                  << std::endl;
    }
}
