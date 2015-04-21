#include <iostream>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::Normal Normal;
typedef pcl::PointCloud<Normal> NormalCloud;
typedef pcl::Label Label;
typedef pcl::PlanarRegion<Point> Plane;
typedef pcl::visualization::PCLVisualizer PCLVisualizer;
typedef boost::shared_ptr<PCLVisualizer> Viewer;

namespace utils {

    struct SEG_OPT {
        int minInliers;
        float angThresh;
        float distThresh;
        float tol;
        float spacingThresh;
    };
    
    void printVec(std::vector<float>);

    float volume(Point, Point);

    Point diff(Point, Point);
    
    Point add(Point, Point);
    
    PointCloud::Ptr loadCloud(std::string);
    
    PointCloud::Ptr trimCloud(PointCloud::Ptr, pcl::PointXYZ, pcl::PointXYZ, bool);
    
    NormalCloud::Ptr computeNormals(PointCloud::Ptr);
    
    void displayPlanarRegions (std::vector<Plane, Eigen::aligned_allocator<Plane> >, Viewer); 
               
}
