#include "utils.hpp"

namespace utils {

    void printVec(std::vector<int> v){
        std::cout << "[";
        for (int i = 0; i < v.size(); i++) {
            std::cout << v[i] << " ";
        }
        std:: cout << "]" << std::endl;
    }

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
        std::cout << "Loading " << name <<  "... ";
        PointCloud::Ptr cloudRaw (new PointCloud);
        if (pcl::io::loadPCDFile<Point> (name, *cloudRaw) == -1) {
            PCL_ERROR ("Couldn't read file\n");
            return cloudRaw;
        }

        std::cout << "cloud has " << cloudRaw->points.size() << " points." << std::endl;
        return cloudRaw;

        std::cout << "Removing NaN's... ";
        PointCloud::Ptr cloudTrim (new PointCloud);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloudRaw, *cloudTrim, indices);
        std::cout << "cleaned cloud has " << cloudTrim->points.size() << " points." << std::endl;

        return cloudTrim;

    }
    
    PointCloud::Ptr trimCloud(PointCloud::Ptr cloud, pcl::PointXYZ lower, pcl::PointXYZ upper, bool save=false) { 
        pcl::PassThrough<Point> pass;
        pass.setKeepOrganized(true);
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
            std::cout << "Trimmed cloud has " << cloud->points.size() << " points left." << std::endl;
            if (save) {
                pcl::io::savePCDFile("trimmed.pcd", *cloud);
            }
        } else {
            std::cout << "no points left" << std::endl;
        }
        return cloud;
    }
    
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

    NormalCloud::Ptr computeNormals(PointCloud::Ptr cloud) {
        std::cout << "Computing normals..." << std::endl;
        pcl::NormalEstimation<Point, Normal> ne;
        ne.setInputCloud(cloud);
        
        pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>());
        ne.setSearchMethod(tree);
        NormalCloud::Ptr normals(new NormalCloud);
        //ne.setRadiusSearch(30);
        ne.setRadiusSearch(0.03);
        ne.compute(*normals);
        return normals;
    }
    
    void displayPlanarRegions (std::vector<Plane, Eigen::aligned_allocator<Plane> > regions, Viewer viewer) { 
        char name[1024];
        unsigned char red [6] = {255, 0, 0, 255, 255, 0};
        unsigned char grn [6] = { 0, 255, 0, 255, 0, 255};
        unsigned char blu [6] = { 0, 0, 255, 0, 255, 255};
        PointCloud::Ptr contour (new PointCloud);
        
        
        for (size_t i = 0; i < regions.size (); i++) {
            Eigen::Vector3f centroid = regions[i].getCentroid ();
            Eigen::Vector4f model = regions[i].getCoefficients ();
            Point pt1 = Point(centroid[0], centroid[1], centroid[2]);
            Point pt2 = Point(centroid[0] + (0.5f * model[0]),
                              centroid[1] + (0.5f * model[1]),
                              centroid[2] + (0.5f * model[2]));
                              
            sprintf (name, "normal_%d", unsigned (i));
        
            viewer->addArrow (pt2, pt1, 1.0, 0, 0, false, name);
            contour->points = regions[i].getContour();
            sprintf (name, "plane_%02d", int (i));
        
            pcl::visualization::PointCloudColorHandlerCustom <Point> color (contour, red[i%6], grn[i%6], blu[i%6]);
            if (!viewer->updatePointCloud(contour, color, name)) {
                viewer->addPointCloud (contour, color, name);
                viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
            }
        }
    }
    
}


