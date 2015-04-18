#include <iostream>
#include "utils.hpp"

int main(int argc, char* argv[]) {
    pcl::HarrisKeypoint3D<Point,pcl::PointXYZI> detector;
    detector.setNonMaxSupression (true);
    detector.setRadius (20);
    //detector.setRadiusSearch (100);
    detector.setInputCloud(pc);

    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>());
    detector.compute(*keypoints);

    std::cout << "keypoints detected: " << keypoints->size() << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints3D(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointXYZ tmp;
    double max = 0,min=0;

    for(pcl::PointCloud<pcl::PointXYZI>::iterator i = keypoints->begin(); i!= keypoints->end(); i++){
        tmp = pcl::PointXYZ((*i).x,(*i).y,(*i).z);
        if ((*i).intensity>max ){
            std::cout << (*i) << " coords: " << (*i).x << ";" << (*i).y << ";" << (*i).z << std::endl;
            max = (*i).intensity;
        }
        if ((*i).intensity<min){
            min = (*i).intensity;
        }
        keypoints3D->push_back(tmp);
    }

    std::cout << "maximal responce: "<< max << " min responce:  "<< min<<std::endl;

    //show point cloud
    pcl::visualization::PCLVisualizer viewer ("3D Viewer");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pccolor(pc, 255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> kpcolor(keypoints3D, 255, 0, 0);
    viewer.addPointCloud(pc,pccolor,"testimg.png");
    viewer.addPointCloud(keypoints3D,kpcolor,"keypoints.png");

    while (!viewer.wasStopped ())
    {
        viewer.spinOnce();
        pcl_sleep (0.01);
    } 
