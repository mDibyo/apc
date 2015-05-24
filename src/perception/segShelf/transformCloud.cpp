#include <iostream>
#include "utils.hpp"

Eigen::Matrix4f readTransform(const char* filename) {

    std::ifstream ifs(filename);

    Eigen::Matrix4f result(4,4);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            float k;
            ifs >> k;
            result(i,j) = k;
        }
    }
    return result;
}

int main(int argc, char* argv[]) {

    
    PointCloud::Ptr cloud = utils::loadCloud(argv[1]);
    Eigen::Matrix4f T = readTransform(argv[2]);
    std::cout << T << std::endl;
    
    pcl::transformPointCloud(*cloud, *cloud, T);
    pcl::io::savePCDFile(argv[3], *cloud);
    
}
