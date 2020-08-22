//
// Created by xcmworkharder on 2020-08-21 上午10:03.
//

#ifndef LIDAR_SLAM_IMU_PUBLISHER_H
#define LIDAR_SLAM_IMU_PUBLISHER_H

#include "sensor_msgs/Imu.h"
#include "lidar_slam/sensor_data/imu_data.h"

namespace lidar_slam {
    class IMUPublisher {
    public:
        IMUPublisher(ros::NodeHandle& nh,
                     std::string topic_name,
                     size_t buff_size,
                     std::string frame_id);
        IMUPublisher() = default;

        void Publish(IMUData imu_data);
        bool HasSubscribers();

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        std::string frame_id_;
    };
}

#endif //LIDAR_SLAM_IMU_PUBLISHER_H
