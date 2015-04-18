#include <iostream>

#include "utils.hpp"

int main(int argc, char** argv) {

    PointCloud::Ptr input = utils::loadCloud(argv[1]);
    Point low(0,0,0.25), high(0,0,atoi(argv[3]));
    PointCloud::Ptr scene = utils::trimCloud(input, low, high, true);


    PointCloud::Ptr shelf = utils::loadCloud(argv[2]);
    Point gmin, gmax, gsize, smin, smax, ssize;
    pcl::getMinMax3D(*scene, gmin, gmax);
    pcl::getMinMax3D(*shelf, smin, smax);
    
    gsize = utils::diff(gmin, gmax);
    ssize = utils::diff(smin, smax);
    
    std::cout << gmin << ", " << gmax << std::endl;
    std::cout << smin << ", " << smax << std::endl;
    
    /*
    int STEP = atoi(argv[4]);
    Point best;
    float maxD;
    for (float x = gmin.x; x < gmax.x; x += STEP) {
        for (float y = gmin.y; y < gmax.y; y += STEP) {
            for (float z = gmin.z; z < gmax.z; z += STEP) {
                Point lower(x,y,z);
                Point upper = utils::add(lower, ssize);
                float d = utils::densityWithinBox(scene, lower, upper);
                if (d > maxD) {
                    maxD = d;
                    best = lower;
                }
                std::cout << lower << ", " << d << std::endl;             
            }
        }
    }
    std::cout << "Best: " << best << ", " << maxD << std::endl; */
}
    
    
    
    
    
    
    
