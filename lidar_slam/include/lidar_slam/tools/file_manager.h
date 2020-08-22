//
// Created by xcmworkharder on 2020-08-15 下午9:21.
//

#ifndef LIDAR_SLAM_FILE_MANAGER_H
#define LIDAR_SLAM_FILE_MANAGER_H

#include <string>
#include <iostream>
#include <fstream>

namespace lidar_slam {
    class FileManager {
    public:
        static bool CreateFile(std::ofstream& ofs, std::string file_path);
        static bool InitDirectory(std::string directory_path, std::string use_for);
        static bool CreateDirectory(std::string directory_path, std::string use_for);
        static bool CreateDirectory(std::string directory_path);
    };
}

#endif //LIDAR_SLAM_FILE_MANAGER_H
