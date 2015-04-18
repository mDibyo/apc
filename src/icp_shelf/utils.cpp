#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include "utils.hpp"

namespace utils {

    Point diff(Point lower, Point upper) {
        return Point(upper.x - lower.x,
                     upper.y - lower.y,
                     upper.z - lower.z);
    }
    
    float volume(Point lower, Point upper) {
        Point d = diff(lower, upper);
        return d.x * d.y * d.z;
    }
    
    Point add(Point one, Point two) {
        return Point(one.x + two.x,
                     one.y + two.y,
                     one.z + two.z);
    }

    PointCloud::Ptr loadCloud(std::string name) {
        std::cout << "Loading " << name <<  "..." << std::endl;
        PointCloud::Ptr cloudRaw (new PointCloud);
        if (pcl::io::loadPCDFile<Point> (name, *cloudRaw) == -1) {
            PCL_ERROR ("Couldn't read file\n");
            return cloudRaw;
        }

        std::cout << "Loaded cloud with " << cloudRaw->points.size() << " points." << std::endl;
      
        std::cout << "Removing NaN's..." << std::endl;
        PointCloud::Ptr cloudTrim (new PointCloud);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloudRaw, *cloudTrim, indices);
        std::cout << "Cleaned cloud has " << cloudTrim->points.size() << " points." << std::endl;
        
        return cloudTrim;
    }
    
    PointCloud::Ptr trimCloud(PointCloud::Ptr cloud, Point lower, Point upper, bool save=false) { 
        pcl::PassThrough<Point> pass;
        std::cout << lower << std::endl;
        std::cout << upper << std::endl;
        if (lower.z != upper.z) {
            pass.setInputCloud(cloud);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits (lower.z, upper.z);
            pass.filter (*cloud);
        }
        
        if (lower.y != upper.y) {
            pass.setInputCloud(cloud);
            pass.setFilterFieldName ("y");
            pass.setFilterLimits (lower.y, upper.y);
            pass.filter (*cloud);
        }

        if (lower.x != upper.x) {
            pass.setInputCloud(cloud);
            pass.setFilterFieldName ("x");
            pass.setFilterLimits (lower.x, upper.x);
            pass.filter (*cloud);
        }
        
        if (cloud->points.size() > 0) {
            std::cout << "points left: " << cloud->points.size() << std::endl;
            if (save) {
                std::ostringstream sstr;
                sstr << "trimmed_" << lower.x << 
                                      lower.y << lower.z << ".pcd";
                std::string fname = sstr.str();
                pcl::io::savePCDFile(fname, *cloud);
            }
        } else {
            std::cout << "no points left" << std::endl;
        }
        return cloud;
    }
    
    float densityWithinBox(PointCloud::Ptr cloudIn, Point lower, Point upper) {
        PointCloud::Ptr cloud(new PointCloud);
        pcl::copyPointCloud(*cloudIn, *cloud);
        std::cout << cloudIn->points.size() << ", " << cloud->points.size() << std::endl;
        PointCloud::Ptr trimmed = trimCloud(cloud, lower, upper, true);
        std::cout << cloudIn->points.size() << ", " << cloud->points.size() << std::endl;
        int numPoints = trimmed->points.size();
        Point low, high;
        pcl::getMinMax3D(*trimmed, low, high);
        float vol = utils::volume(low, high);
        return numPoints * vol;
    }
}


