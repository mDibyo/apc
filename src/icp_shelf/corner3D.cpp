#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/keypoints/harris_keypoint3D.h>
#include "utils.hpp"

int main(int argc, char* argv[]) {
    PointCloud::Ptr pc = utils::loadCloud(argv[1]);
    
    pcl::HarrisKeypoint3D<Point,Point> detector;
    detector.setNonMaxSupression (true);
    detector.setRadius (20);
    detector.setInputCloud(pc);

    PointCloud::Ptr keypoints(new PointCloud::Ptr);
    detector.compute(*keypoints);
    std::cout << "keypoints detected: " << keypoints->size() << std::endl;
}


