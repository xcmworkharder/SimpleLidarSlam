#include "lidar_slam/subscriber/gnss_subscriber.h"
#include "glog/logging.h"

namespace lidar_slam {

    GNSSSubscriber::GNSSSubscriber(ros::NodeHandle &nh, std::string topic_name,
                                   size_t buff_size) : nh_(nh) {
        subscriber_ = nh_.subscribe(topic_name, buff_size,
                                    &GNSSSubscriber::msg_callback, this);
    }

    void GNSSSubscriber::msg_callback(
            const sensor_msgs::NavSatFixConstPtr &nav_sat_fix_ptr) {
        GNSSData gnssData;
        gnssData.time = nav_sat_fix_ptr->header.stamp.toSec();
        gnssData.latitude = nav_sat_fix_ptr->latitude;
        gnssData.longitude = nav_sat_fix_ptr->longitude;
        gnssData.altitude = nav_sat_fix_ptr->altitude; // 原来代码写的longitude
        gnssData.status = nav_sat_fix_ptr->status.status;
        gnssData.service = nav_sat_fix_ptr->status.service;

        new_gnss_data_.push_back(gnssData);
    }

    void GNSSSubscriber::ParseData(std::deque<GNSSData>& gnss_data_buff) {
        if (new_gnss_data_.size() > 0) {
            gnss_data_buff.insert(gnss_data_buff.end(),
                                  new_gnss_data_.begin(), new_gnss_data_.end());
            new_gnss_data_.clear();
        }
    }
}