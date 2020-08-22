//
// Created by xcmworkharder on 2020-08-15 上午8:52.
//
#include "lidar_slam/sensor_data/velocity_data.h"
#include "glog/logging.h"

namespace lidar_slam {
    bool VelocityData::SyncData(std::deque<VelocityData>& UnsyncedData,
                               std::deque<VelocityData>& SyncedData,
                               double sync_time) {
        while (UnsyncedData.size() >= 2) {
            if (UnsyncedData.front().time > sync_time)
                return false;
            if (UnsyncedData[1].time < sync_time) {
                UnsyncedData.pop_front();
                continue;
            }
            if (sync_time - UnsyncedData.front().time > 0.2) {
                UnsyncedData.pop_front();
                break;
            }
            if (UnsyncedData[1].time - sync_time > 0.2) {
                UnsyncedData.pop_front();
                break;
            }
            break;
        }
        if (UnsyncedData.size() < 2)
            return false;

        VelocityData front_data = UnsyncedData[0];
        VelocityData back_data = UnsyncedData[1];
        VelocityData synced_data;

        double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
        double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
        synced_data.time = sync_time;
        synced_data.linear_velocity.x = front_data.linear_velocity.x * front_scale + back_data.linear_velocity.x * back_scale;
        synced_data.linear_velocity.y = front_data.linear_velocity.y * front_scale + back_data.linear_velocity.y * back_scale;
        synced_data.linear_velocity.z = front_data.linear_velocity.z * front_scale + back_data.linear_velocity.z * back_scale;
        synced_data.angular_velocity.x = front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
        synced_data.angular_velocity.y = front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
        synced_data.angular_velocity.z = front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;

        SyncedData.push_back(synced_data);

        return true;
    }

    void VelocityData::TransformCoordinate(Eigen::Matrix4f transform_matrix) {
        Eigen::Matrix4d matrix = transform_matrix.cast<double>();
        Eigen::Matrix3d t_R = matrix.block<3, 3>(0, 0);
        Eigen::Vector3d w(angular_velocity.x, angular_velocity.y, angular_velocity.z);
        Eigen::Vector3d v(linear_velocity.x, linear_velocity.y, linear_velocity.z);
        w = t_R * w;
        v = t_R * v;

        Eigen::Vector3d r(matrix(0, 3), matrix(1, 3), matrix(2, 3));
        Eigen::Vector3d delta_v;
        delta_v(0) = w(1) * r(2) - w(2) * r(1);
        delta_v(1) = w(2) * r(0) - w(0) * r(2);
        delta_v(2) = w(0) * r(1) - w(1) * r(0); // 原来bug：w(1) * r(1) - w(1) * r(0);
        v += delta_v;

        angular_velocity.x = w(0);
        angular_velocity.y = w(1);
        angular_velocity.z = w(2);
        linear_velocity.x = v(0);
        linear_velocity.y = v(1);
        linear_velocity.z = v(2);
    }
}
