#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv) {

  std::cout << "Loading " << argv[1] <<  "..." << std::endl;
  PointCloud::Ptr cloudRaw (new PointCloud);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloudRaw) == -1) {
      PCL_ERROR ("Couldn't read file\n");
      return -1;
  }
  std::cout << "Loaded cloud with " << cloudRaw->points.size() << " points." << std::endl;
  
  std::cout << "Removing NaN's..." << std::endl;
  PointCloud::Ptr cloudTrim (new PointCloud);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloudRaw, *cloudTrim, indices);
  std::cout << "Trimmed cloud has " << cloudTrim->points.size() << " points." << std::endl;
  
  pcl::io::savePLYFile("trimmed.ply", *cloudTrim);
  std::cout << "wrote trimmed cloud to file" << std::endl;
  
  std::cout << "Loading shelf model..." << std::endl;
  PointCloud::Ptr cloudShelf (new PointCloud);
  pcl::io::loadPLYFile(argv[2], *cloudShelf);
  std::cout << "Loaded shelf cloud with " << cloudShelf->points.size() << " points." << std::endl;
  
  std::cout << "Beginning ICP..." << std::endl;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud(cloudTrim);
  icp.setInputTarget(cloudShelf);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  std::cout << "Convergence:" << icp.hasConverged() << ", Score: " << icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  
}
