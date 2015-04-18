#include <iostream>
#include "utils.hpp"


int main(int argc, char** argv) {

    PointCloud::Ptr input = utils::loadCloud(argv[1]);
    Point min, max;
    pcl::getMinMax3D(*input, min, max);
    std::cout << min << ", " << max << std::endl;
    Point low(atoi(argv[2]),atoi(argv[3]),atoi(argv[4]));
    Point high(atoi(argv[5]), atoi(argv[6]), atoi(argv[7]));
    PointCloud::Ptr cloud = utils::trimCloud(input, low, high, true);
    
}
