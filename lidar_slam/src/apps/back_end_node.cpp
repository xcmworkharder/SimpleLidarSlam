//
// Created by xcmworkharder on 2020-08-21 上午10:26.
//
#include <ros/ros.h>
#include "glog/logging.h"

#include "lidar_slam/global_defination/global_defination.h"
#include "lidar_slam/mapping/back_end/back_end_flow.h"

using namespace lidar_slam;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "back_end_node");
    ros::NodeHandle nh;

    std::shared_ptr<BackEndFlow> back_end_flow_ptr = std::make_shared<BackEndFlow>(nh);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        back_end_flow_ptr->Run();
        rate.sleep();
    }

    return 0;
}
