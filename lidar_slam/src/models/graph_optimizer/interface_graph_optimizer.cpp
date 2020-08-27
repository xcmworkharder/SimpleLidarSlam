//
// Created by xcmworkharder on 2020-08-26 下午4:56.
//
#include "lidar_slam/models/graph_optimizer/interface_graph_optimizer.h"

namespace lidar_slam {
    void GraphOptimizerInterface::SetMaxIterationsNum(int max_iterations_num) {
        max_iterations_num_ = max_iterations_num;
    }
}
