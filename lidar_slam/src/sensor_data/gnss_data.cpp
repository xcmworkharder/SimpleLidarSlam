#include "lidar_slam/sensor_data/gnss_data.h"
#include "glog/logging.h"

// 静态变量初始化
bool lidar_slam::GNSSData::origin_position_inited = false;
GeographicLib::LocalCartesian lidar_slam::GNSSData::geo_converter;

namespace lidar_slam {

    void GNSSData::InitOriginPosition() {
        geo_converter.Reset(latitude, longitude, altitude);
        origin_position_inited = true;
    }

    void GNSSData::UpdateXYZ() {
        if (!origin_position_inited) {
            LOG(WARNING) << "GeoConverter has not set origin position";
        }
        geo_converter.Forward(latitude, longitude, altitude,
                              local_E, local_N, local_U);
    }
}
