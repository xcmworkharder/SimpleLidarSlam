#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_IMU_SUBSCRIBER_H_
#define LIDAR_LOCALIZATION_SUBSCRIBER_IMU_SUBSCRIBER_H_

#include <deque>
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "lidar_slam/sensor_data/imu_data.h"

namespace lidar_localization {
    class IMUSubscriber {
    public:
        IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
        IMUSubscriber() = default;
        void ParseData(std::deque<IMUData>& imu_data_buff);

    private:
        void msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;

        std::deque<IMUData> new_imu_data_;
    };
}
#endif