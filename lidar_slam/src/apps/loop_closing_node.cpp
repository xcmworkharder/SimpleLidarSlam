//
// Created by xcmworkharder on 2020-08-27 下午4:59.
//
#include <ros/ros.h>
#include "glog/logging.h"

#include "lidar_slam/global_defination/global_defination.h"
#include "lidar_slam/mapping/loop_closing/loop_closing_flow.h"

using namespace lidar_slam;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "loop_closing_node");
    ros::NodeHandle nh;

    std::shared_ptr<LoopClosingFlow> loop_closing_flow_ptr = std::make_shared<LoopClosingFlow>(nh);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        loop_closing_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}
