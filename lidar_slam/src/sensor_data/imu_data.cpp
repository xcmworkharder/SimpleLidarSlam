//
// Created by xcmworkharder on 2020-08-15 上午8:52.
//
#include "lidar_slam/sensor_data/imu_data.h"
#include <cmath>
#include "glog/logging.h"

namespace lidar_slam {
    Eigen::Matrix3f IMUData::GetOrientationMatrix() {
        Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
        Eigen::Matrix3f matrix = q.matrix().cast<float>();

        return matrix;
    }

    bool IMUData::SyncData(std::deque<IMUData>& UnsyncedData,
                           std::deque<IMUData>& SyncedData, double sync_time) {
        /// 找到sync_time左右相邻两个数据,需确保与同步时刻的差距不能太大
        while (UnsyncedData.size() > 2) {
            if (UnsyncedData.front().time > sync_time) {
                return false;
            }
            /// 如果第二个数据都小于同步时刻，则第一个数据直接丢弃
            if (UnsyncedData[1].time < sync_time) {
                UnsyncedData.pop_front();
                continue;
            }
            /// 如果与左邻居差距太大, 丢弃第一个数据,todo:这里是否应以该返回false
            if (sync_time - UnsyncedData.front().time > 0.2) {
                UnsyncedData.pop_front();
                break;
            }
            /// 如果与有邻居差距太大，丢掉第一个数据, todo:第二个数据也应该丢掉，这里是否应该返回false
            if (UnsyncedData[1].time - sync_time > 0.2) {
                UnsyncedData.pop_front();
                break;
            }
            break;
        }
        /// 如果数据不足两个，没法插值
        if (UnsyncedData.size() < 2) {
            return false;
        }

        IMUData front_data = UnsyncedData[0];
        IMUData back_data = UnsyncedData[1];
        IMUData synced_data;

        double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
        double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
        synced_data.time = sync_time;
        synced_data.linear_acceleration.x = front_data.linear_acceleration.x * front_scale + back_data.linear_acceleration.x * back_scale;
        synced_data.linear_acceleration.y = front_data.linear_acceleration.y * front_scale + back_data.linear_acceleration.y * back_scale;
        synced_data.linear_acceleration.z = front_data.linear_acceleration.z * front_scale + back_data.linear_acceleration.z * back_scale;
        synced_data.angular_velocity.x = front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
        synced_data.angular_velocity.y = front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
        synced_data.angular_velocity.z = front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;
        /// 四元数插值有线性插值和球面插值，球面插值更准确，但是两个四元数差别不大是，二者精度相当
        /// 由于是对相邻两时刻姿态插值，姿态差比较小，所以可以用线性插值
        synced_data.orientation.x = front_data.orientation.x * front_scale + back_data.orientation.x * back_scale;
        synced_data.orientation.y = front_data.orientation.y * front_scale + back_data.orientation.y * back_scale;
        synced_data.orientation.z = front_data.orientation.z * front_scale + back_data.orientation.z * back_scale;
        synced_data.orientation.w = front_data.orientation.w * front_scale + back_data.orientation.w * back_scale;
        /// 线性插值后要进行归一化
        synced_data.orientation.Normlize();

        SyncedData.push_back(synced_data);

        return true;
    }
}

