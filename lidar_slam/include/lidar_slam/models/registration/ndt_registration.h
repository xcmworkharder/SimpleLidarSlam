//
// Created by xcmworkharder on 2020-08-13 下午5:28.
//

#ifndef LIDAR_SLAM_NDT_REGISTRATION_H
#define LIDAR_SLAM_NDT_REGISTRATION_H

#include <pcl/registration/ndt.h>
#include "lidar_slam/models/registration/registration_interface.h"

namespace lidar_slam {
    class NDTRegistration : public RegistrationInterface {
    public:
        NDTRegistration(const YAML::Node& node);
        NDTRegistration(float res, float step_size, float trans_eps, int max_iter);

        bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
        bool ScanMatch(const CloudData::CLOUD_PTR& input_source,
                       const Eigen::Matrix4f& predict_pose,
                       CloudData::CLOUD_PTR& result_cloud_ptr,
                       Eigen::Matrix4f& result_pose) override;

    private:
        bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter);

    private:
        pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr ndt_ptr_;
    };
}


#endif //LIDAR_SLAM_NDT_REGISTRATION_H
