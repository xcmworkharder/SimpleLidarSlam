//
// Created by xcmworkharder on 2020-08-15 上午8:48.
//

#ifndef LIDAR_SLAM_VELOCITY_DATA_H
#define LIDAR_SLAM_VELOCITY_DATA_H

#include <deque>
#include <Eigen/Dense>

namespace lidar_slam {
    class VelocityData {
    public:
        struct LinearVelocity {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
        };

        struct AngularVelocity {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
        };

        double time = 0.0;
        LinearVelocity linear_velocity;
        AngularVelocity angular_velocity;

    public:
        static bool SyncData(std::deque<VelocityData>& UnsyncedData,
                            std::deque<VelocityData>& SyncedData, double sync_time);
        void TransformCoordinate(Eigen::Matrix4f transform_matrix);
    };
}

#endif //LIDAR_SLAM_VELOCITY_DATA_H
