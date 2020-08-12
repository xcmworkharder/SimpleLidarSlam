#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_GNSS_SUBSCRIBER_H_
#define LIDAR_LOCALIZATION_SUBSCRIBER_GNSS_SUBSCRIBER_H_

#include <deque>
#include <ros/ros.h>
#include "sensor_msgs/NavSatFix.h"
#include "lidar_slam/sensor_data/gnss_data.h"

namespace lidar_localization {
    class GNSSSubscriber {
    public:
        GNSSSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
        GNSSSubscriber() = default;
        void ParseData(std::deque<GNSSData>& gnss_data_buff);

    private:
        void msg_callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;

        std::deque<GNSSData> new_gnss_data_;
    };
}
#endif