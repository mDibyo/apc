#include <iostream>
#include <math.h>
#include "utils.hpp"


float in_to_mm = 0.0254;
float SHELF_YSTEPS[] = {in_to_mm*9, in_to_mm*19, in_to_mm*28, in_to_mm*37, in_to_mm*47}; // actual values: {780, 1070, 1300, 1520}
float SHELF_WIDTH = 0.9; // meters
float SHELF_DEPTH = 0.226; // meters
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
                                    Eigen::Matrix4f shelfMat, PointCloud::Ptr segmented, bool display) {   
    
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
        if (fabs(c.values[2]) < opt.tol) {
            if (fabs(c.values[0]) > fabs(c.values[1])) {
                front.push_back(i);
                valid.push_back(i);
            } else {
                side.push_back(i);
                valid.push_back(i);
            }
        }
        if (fabs(c.values[2]) > (1 - opt.tol)) {
            up.push_back(i);
            valid.push_back(i);
        } 
    }

    printPlaneInfo(coeffs);
     
    std::cout << "up: ";
    utils::printVec(up);
    
    std::cout << "side: ";
    utils::printVec(side);
    
    std::cout << "front: ";
    utils::printVec(front);
    
   
    
    /* find two planes approx SHELF_WIDTH apart
     * find the angle that their normal needs to be rotated about y so that it becomes +X
     * average plane centroids and take x to get shelf x
     */
    Eigen::Vector3f sideNormal(0,0,0), shelfTrans(0,0,0);
    for (int i = 0; i < side.size(); i++) {
        for (int j = 0; j < side.size(); j++) {
            float dist = fabs(coeffs[side[i]].values[1]*coeffs[side[i]].values[3] - coeffs[side[j]].values[1]*coeffs[side[j]].values[3]);
            std::cout << side[i] << "," << side[j] << ": " << dist << std::endl;
            if (fabs(dist - SHELF_WIDTH) < opt.spacingThresh) {
                std::cout << "picked " << side[i] << "," << side[j] 
                          << " as sides, dist=" << dist << std::endl;
                for (int k = 0; k < 3; k++) {
                    if (coeffs[side[i]].values[1] > 0) {
                        sideNormal[k] += coeffs[side[i]].values[k];
                    } else {
                        sideNormal[k] -= coeffs[side[i]].values[k];
                    }
                    if (coeffs[side[j]].values[1] > 0) {
                        sideNormal[k] += coeffs[side[j]].values[k];
                    } else {
                        sideNormal[k] -= coeffs[side[j]].values[k];
                    }
                }
                sideNormal /= 2;
                
                Eigen::Vector3f center = regions[side[i]].getCentroid() + regions[side[j]].getCentroid();
                shelfTrans[1] = center[1]/2;
                i = side.size();
                j = side.size();
            }
        }
    }
    
    // average depth of planes facing front to get Z
    for (int i = 0; i < front.size(); i++) {
        //std::cout << front[i] << std::endl;
        //std::cout << regions[front[i]].getCentroid() << std::endl;
        shelfTrans[0] += regions[front[i]].getCentroid()[0];
    }
    shelfTrans[0] /= front.size();
    std::cout << "depth: " << shelfTrans[0] << std::endl;
    /*
     * check if y values are close to known shelf heights
     * average displacement is shelf y
     */
    int shelvesFound = 0;
    for (int i = 0; i < sizeof(SHELF_YSTEPS)/4; i++) {
        for (int j = 0; j < up.size(); j++) {
            float dist = coeffs[up[j]].values[3] - SHELF_YSTEPS[i];
            if (fabs(dist) < 0.02) {
                std::cout << "height match:\t" << up[j] << "," << i << ":\t"
                          << coeffs[up[j]].values[3] << "," << SHELF_YSTEPS[i] << std::endl;
                shelfTrans[2] += dist;
                shelvesFound++;
            }
        }
    }
    shelfTrans[1] /= shelvesFound;
    
    // may need to fix signs depending on coordinate system
    float theta = acos(sideNormal[1] / sideNormal.norm()); // shelf yaw   
    std::cout << "shelf yaw: " << theta << std::endl;
    shelfMat << cos(theta), -sin(theta),           0,   SHELF_DEPTH + shelfTrans[0],
                sin(theta),  cos(theta),           0,                 shelfTrans[1],
                0,           0,                    1,                 shelfTrans[2],
                0,           0,                    0,                             1;
                
                
    std::cout << "Shelf transform is: " << std::endl << shelfMat << std::endl;            
        
        
    // trim the cloud XY to shelf
    pcl::copyPointCloud(*cloud, *segmented);
    pcl::PointXYZ lower(shelfMat(0,3) - SHELF_WIDTH, shelfMat(1,3), -6*SHELF_DEPTH),
                  upper(shelfMat(0,3) + SHELF_WIDTH, shelfMat(1,3) + SHELF_HEIGHT, 0);
    utils::trimCloud(segmented, lower, upper, false);
    try {
        pcl::io::savePCDFile("cropped.pcd", *segmented);
    } catch (pcl::IOException e) {
    }
    
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
    try {
        pcl::io::savePCDFile("segmented.pcd", *segmented);
    } catch (pcl::IOException e) {
    }
    
    // view planes found
    if (display) {
        Viewer viewer(new PCLVisualizer ("viewer"));
        utils::displayPlanarRegions(regions, viewer);
        viewer->addPointCloud(cloud);
        while (!viewer->wasStopped ()) {
            viewer->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
    }
    return shelfMat;
}



int main(int argc, char* argv[]) {

    Eigen::Matrix4f cameraToBase = utils::readTransform(argv[2]);   

    /*
    cameraToBase << -1,  0,     0,      0,
                    0, -0.72, -0.69, 1720,
                    0,  0.69, -0.72,    0,
                    0,  0,     0,       1;
    */

    PointCloud::Ptr raw = utils::loadCloud(argv[1]);
    // transform cloud into base_link frame
    pcl::transformPointCloud(*raw, *raw, cameraToBase);
    PointCloud::Ptr cloud = utils::trimCloud(raw, pcl::PointXYZ(0,0,0), pcl::PointXYZ(2,0,0), false);
    pcl::io::savePCDFile("transformed.pcd", *cloud);

    bool display = argv[3];
    utils::SEG_OPT options;
    if (argc > 5) {
        options.minInliers = atoi(argv[4]);
        options.angThresh = atof(argv[5]);
        options.distThresh = atof(argv[6]);
        options.tol = atof(argv[7]);
        options.spacingThresh = 0.05f;
    } else {
        options.minInliers = 50;
        options.angThresh = 0.05;
        options.distThresh = 0.05;
        options.tol = 0.1;
        options.spacingThresh = 0.05f;
    }
    
    
    PointCloud::Ptr segmented(new PointCloud);
    Eigen::Matrix4f shelfTransform;
    shelfTransform = findAndSegmentShelf(cloud, options, shelfTransform, segmented, display);
    
    ofstream ofs("shelf_pose.txt");
    ofs << shelfTransform << std::endl;
    ofs.close();
    
    ofstream ofss("../../../data/transforms/shelf_pose.txt");
    ofss << shelfTransform << std::endl;
    ofss.close();
      
}


