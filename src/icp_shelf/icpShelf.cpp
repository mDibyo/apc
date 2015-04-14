#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

class FeatureCloud {
  public:
    FeatureCloud () :
      search_method_xyz_ (new SearchMethod),
      normal_radius_ (0.02f),
      feature_radius_ (0.02f)
    {}
    ~FeatureCloud () {}

    void setInputCloud (PointCloud::Ptr xyz) {
      xyz_ = xyz;
      processInput();
    }

    PointCloud::Ptr getPointCloud() const {
      return (xyz_);
    }

    SurfaceNormals::Ptr getSurfaceNormals () const {
      return (normals_);
    }

    // Get a pointer to the cloud of feature descriptors
    LocalFeatures::Ptr getLocalFeatures () const {
      return (features_);
    }

  protected:
    void processInput () {
      computeSurfaceNormals ();
      computeLocalFeatures ();
    }

    void computeSurfaceNormals () {
      normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
      norm_est.setInputCloud (xyz_);
      norm_est.setSearchMethod (search_method_xyz_);
      norm_est.setRadiusSearch (normal_radius_);
      norm_est.compute (*normals_);
    }

    void computeLocalFeatures ()
    {
      features_ = LocalFeatures::Ptr (new LocalFeatures);

      pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
      fpfh_est.setInputCloud (xyz_);
      fpfh_est.setInputNormals (normals_);
      fpfh_est.setSearchMethod (search_method_xyz_);
      fpfh_est.setRadiusSearch (feature_radius_);
      fpfh_est.compute (*features_);
    }

  private:
    // Point cloud data
    PointCloud::Ptr xyz_;
    SurfaceNormals::Ptr normals_;
    LocalFeatures::Ptr features_;
    SearchMethod::Ptr search_method_xyz_;

    // Parameters
    float normal_radius_;
    float feature_radius_;
};

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

    std::cout << "Loading shelf model..." << std::endl;
    PointCloud::Ptr cloudShelf (new PointCloud);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[2], *cloudShelf) == -1) {
        PCL_ERROR ("Couldn't read file\n");
        return -1;
    }
    std::cout << "Loaded shelf cloud with " << cloudShelf->points.size() << " points." << std::endl;
  
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
  
  
  /*
  std::cout << "Beginning ICP..." << std::endl;
  pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaximumIterations(5);
  icp.setTransformationEpsilon (1e-4);
  icp.setEuclideanFitnessEpsilon (1);
  icp.setInputCloud(cloudTrim);
  icp.setInputTarget(cloudShelf);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  std::cout << "Convergence:" << icp.hasConverged() << ", Score: " << icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  
  std::ofstream outstream;
  outstream.open("transform.txt");
  outstream << icp.getFinalTransformation();
  outstream.close();
  std::cout << "wrote to transform.txt" << std::endl; */
}
