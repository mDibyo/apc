#include <iostream>
#include <math.h>
#include "utils.hpp"

float SHELF_YSTEPS[] = {25.4*9, 25.4*19, 25.4*28, 25.4*37, 25.4*47}; // actual values: {780, 1070, 1300, 1520}
float SHELF_WIDTH = 900;
float SHELF_DEPTH = 216;
float SHELF_HEIGHT = SHELF_YSTEPS[4];

void printPlaneInfo(std::vector<pcl::ModelCoefficients> coeffs) {
    for (int i = 0; i < coeffs.size(); i++) {
        pcl::ModelCoefficients coeff = coeffs[i];
        std::cout << "Plane " << i << ": ["
                  << coeff.values[0] << ", "
                  << coeff.values[1] << ", "
                  << coeff.values[2] << ", "
                  << coeff.values[3] << "]" << std::endl;
    }
}

Eigen::Matrix4f findAndSegmentShelf(PointCloud::Ptr cloud, utils::SEG_OPT opt,
                                    Eigen::Matrix4f shelfMat, PointCloud::Ptr segmented) {   
    
    // estimate normals
    NormalCloud::Ptr normals = utils::computeNormals(cloud);
    
    // find all planes in cloud
    std::cout << "Finding planes... ";
    pcl::OrganizedMultiPlaneSegmentation<Point, Normal, Label> mps;
    mps.setMinInliers(opt.minInliers);
    mps.setAngularThreshold(opt.angThresh);
    mps.setDistanceThreshold(opt.distThresh);
    mps.setInputNormals(normals);
    mps.setInputCloud(cloud);
    
    std::vector<Plane, Eigen::aligned_allocator<Plane> > regions;
    std::vector<pcl::ModelCoefficients> coeffs;
    std::vector<pcl::PointIndices> inlierInd;
    pcl::PointCloud<Label>::Ptr labels (new pcl::PointCloud<Label>);
    std::vector<pcl::PointIndices> labelInd;
    std::vector<pcl::PointIndices> boundaryInd;

    mps.setInputNormals (normals);
    mps.setInputCloud (cloud); 
    mps.segmentAndRefine (regions, coeffs, inlierInd, labels, labelInd, boundaryInd);
    
    std::cout << regions.size() << std::endl;
    
    // large y means flat shelf, no y means bigger of [x,z]
    std::vector<int> up, side, front, valid;
  
    for (int i = 0; i < coeffs.size(); i++) {
        pcl::ModelCoefficients c = coeffs[i];
        if (fabs(c.values[1]) < opt.tol) {
            if (fabs(c.values[2]) > fabs(c.values[0])) {
                front.push_back(i);
                valid.push_back(i);
            } else {
                side.push_back(i);
                valid.push_back(i);
            }
        }
        if (fabs(c.values[1]) > (1 - opt.tol)) {
            up.push_back(i);
            valid.push_back(i);
        } 
    }

    /*
    utils::printVec(up);
    utils::printVec(side);
    utils::printVec(front);
    printPlaneInfo(coeffs);
    */
    
    /* find two planes approx SHELF_WIDTH apart
     * find the angle that their normal needs to be rotated about y so that it becomes +X
     * average plane centroids and take x to get shelf x
     */
    Eigen::Vector3f sideNormal(0,0,0), shelfTrans(0,0,0);
    for (int i = 0; i < side.size(); i++) {
        for (int j = 0; j < side.size(); j++) {
            float dist = fabs(coeffs[side[i]].values[3] - coeffs[side[j]].values[3]);
            if (fabs(dist - SHELF_WIDTH) < opt.spacingThresh) {
                for (int k = 0; k < 3; k++) {
                    if (coeffs[side[i]].values[0] > 0) {
                        sideNormal[k] += coeffs[side[i]].values[k];
                    } else {
                        sideNormal[k] -= coeffs[side[i]].values[k];
                    }
                    if (coeffs[side[j]].values[0] > 0) {
                        sideNormal[k] += coeffs[side[j]].values[k];
                    } else {
                        sideNormal[k] -= coeffs[side[j]].values[k];
                    }
                }
                sideNormal /= 2;
                
                Eigen::Vector3f center = regions[side[i]].getCentroid() + regions[side[j]].getCentroid();
                shelfTrans[0] = center[0]/2;
                i = side.size();
                j = side.size();
            }
        }
    }
    
    // average depth of planes facing front to get Z
    for (int i = 0; i < front.size(); i++) {
        shelfTrans[2] += regions[front[i]].getCentroid()[2];
    }
    shelfTrans[2] /= front.size();
    
    /*
     * check if y values are close to known shelf heights
     * average displacement is shelf y
     */
    int shelvesFound = 0;
    for (int i = 0; i < sizeof(SHELF_YSTEPS)/4; i++) {
        for (int j = 0; j < up.size(); j++) {
            float dist = coeffs[up[j]].values[3] - SHELF_YSTEPS[i];
            if (fabs(dist) < opt.spacingThresh) {
                shelfTrans[1] += dist;
                shelvesFound++;
            }
        }
    }
    shelfTrans[1] /= shelvesFound;
    
    // may need to fix signs depending on coordinate system
    float theta = -acos(sideNormal[0] / sideNormal.norm()); // shelf yaw   
    shelfMat << cos(theta), 0,           -sin(theta),                -shelfTrans[0],
                0,          1,                     0,                 shelfTrans[1],
                sin(theta), 0,            cos(theta),                 shelfTrans[2],
                0,          0,                     0,                             1;
                
                
    std::cout << "Shelf transform is: " << std::endl << shelfMat << std::endl;            
        
        
    // trim the cloud XY to shelf
    pcl::copyPointCloud(*cloud, *segmented);
    pcl::PointXYZ lower(shelfMat(0,3) - SHELF_WIDTH, shelfMat(1,3), -6*SHELF_DEPTH),
                  upper(shelfMat(0,3) + SHELF_WIDTH, shelfMat(1,3) + SHELF_HEIGHT, 0);
    utils::trimCloud(segmented, lower, upper, false);
    
    pcl::io::savePCDFile("cropped.pcd", *segmented);
    
    // remove planes and save
    pcl::ExtractIndices<Point> extract;
    extract.setInputCloud(segmented);
    extract.setKeepOrganized(true);
    
    for (int i = 0; i < valid.size(); i++) {
        pcl::PointIndices::Ptr ind(new pcl::PointIndices(inlierInd[valid[i]]));
        extract.setIndices(ind);
        extract.setNegative(true);
        extract.setKeepOrganized(false);
        extract.filterDirectly(segmented);
    }
    std::vector<int> nanIdx;
    pcl::removeNaNFromPointCloud(*segmented, *segmented, nanIdx);
    pcl::io::savePCDFile("segmented.pcd", *segmented);
    
    
    // view planes found    
    Viewer viewer(new PCLVisualizer ("viewer"));
    utils::displayPlanarRegions(regions, viewer);
    viewer->addPointCloud(cloud);
    while (!viewer->wasStopped ()) {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    
    return shelfMat;
}

int main(int argc, char* argv[]) {

    Eigen::Matrix4f cameraToBase;
    cameraToBase << -1,  0,     0,      0,
                    0, -0.72, -0.69, 1720,
                    0,  0.69, -0.72,    0,
                    0,  0,     0,       1;

    PointCloud::Ptr cloud = utils::loadCloud(argv[1]);
    // transform cloud into base_link frame
    pcl::transformPointCloud(*cloud, *cloud, cameraToBase);
    
    utils::SEG_OPT options;
    if (argc > 5) {
        options.minInliers = atoi(argv[2]);
        options.angThresh = atof(argv[3]);
        options.distThresh = atof(argv[4]);
        options.tol = atof(argv[5]);
        options.spacingThresh = 10.0f;
    } else {
        options.minInliers = 50;
        options.angThresh = 0.05;
        options.distThresh = 0.05;
        options.tol = 0.1;
        options.spacingThresh = 10.0f;
    }
    
    Eigen::Matrix4f shelfTransform;
    PointCloud::Ptr segmented(new PointCloud);
    findAndSegmentShelf(cloud, options, shelfTransform, segmented);
}
