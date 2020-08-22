//
// Created by xcmworkharder on 2020-08-21 上午10:46.
//
#include <ros/ros.h>
#include "glog/logging.h"

#include <lidar_slam/saveMap.h>
#include "lidar_slam/global_defination/global_defination.h"
#include "lidar_slam/mapping/viewer/viewer_flow.h"

using namespace lidar_slam;

std::shared_ptr<ViewerFlow> _viewer_flow_ptr;
bool _need_save_map = false;

bool save_map_callback(saveMap::Request& request, saveMap::Response& response) {
    _need_save_map = true;
    response.succeed = true;
    return response.succeed;
}

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "viewer_node");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("save_map", save_map_callback);
    std::shared_ptr<ViewerFlow> _viewer_flow_ptr = std::make_shared<ViewerFlow>(nh);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        _viewer_flow_ptr->Run();
        if (_need_save_map) {
            _need_save_map = false;
            _viewer_flow_ptr->SaveMap();
        }

        rate.sleep();
    }

    return 0;
}
