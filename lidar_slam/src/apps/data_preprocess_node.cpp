//
// Created by xcmworkharder on 2020-08-21 上午10:27.
//
#include <ros/ros.h>
#include "glog/logging.h"

#include "lidar_slam/global_defination/global_defination.h"
#include "lidar_slam/data_preprocess/data_preprocess_flow.h"

using namespace lidar_slam;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "data_preprocess_node");
    ros::NodeHandle nh;

    std::string cloud_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");

    std::shared_ptr<DataPreprocessFlow> data_preprocess_flow_ptr =
            std::make_shared<DataPreprocessFlow>(nh, cloud_topic);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        data_preprocess_flow_ptr->Run();
        rate.sleep();
    }

    return 0;
}
