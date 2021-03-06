//
// Created by xcmworkharder on 2020-08-21 上午10:26.
//
#include <ros/ros.h>
#include "glog/logging.h"

#include <lidar_slam/optimizeMap.h>
#include "lidar_slam/global_defination/global_defination.h"
#include "lidar_slam/mapping/back_end/back_end_flow.h"

using namespace lidar_slam;

std::shared_ptr<BackEndFlow> _back_end_flow_ptr;
bool _need_optimize_map = false;

bool optimize_map_callback(optimizeMap::Request& request, optimizeMap::Response& response) {
    _need_optimize_map = true;
    response.succeed = true;
    return response.succeed;
}

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "back_end_node");
    ros::NodeHandle nh;

    std::string cloud_topic, odom_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");
    nh.param<std::string>("odom_topic", odom_topic, "/laser_odom");

    ros::ServiceServer service = nh.advertiseService("optimize_map", optimize_map_callback);
    _back_end_flow_ptr = std::make_shared<BackEndFlow>(nh, cloud_topic, odom_topic);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        _back_end_flow_ptr->Run();

        if (_need_optimize_map) {
            _back_end_flow_ptr->ForceOptimize();
            _need_optimize_map = false;
        }

        rate.sleep();
    }

    return 0;
}
