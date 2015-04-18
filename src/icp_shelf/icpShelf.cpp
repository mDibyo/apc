#include <iostream>
#include <pcl/registration/icp.h>

#include "utils.hpp"

int main(int argc, char** argv) {

  PointCloud::Ptr raw = utils::loadCloud(argv[1]);
  Point low(0,0,0);
  Point high(0,0,atoi(argv[3]));
  PointCloud::Ptr scene = utils::trimCloud(raw, low, high, false);
  PointCloud::Ptr shelf = utils::loadCloud(argv[2]);
  
  
  
  std::cout << "Beginning ICP..." << std::endl;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaxCorrespondenceDistance(atof(argv[4]));
  icp.setInputCloud(scene);
  icp.setInputTarget(shelf);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  std::cout << "Convergence:" << icp.hasConverged() << ", Score: " << icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  
  std::ofstream outstream;
  outstream.open("icp.txt");
  outstream << icp.getFinalTransformation();
  outstream.close();
  std::cout << "wrote to icp.txt" << std::endl;
}
