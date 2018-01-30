#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/video/tracking.hpp>

#include "Configurations.h"
#include "CloudProjection.h"

int main (int argc, char** argv) {

    Configurations::getInstance()->readConfig();
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    std::vector<int> ply_filenames;
    ply_filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");
    if (ply_filenames.size () != 2) {
        std::cout << "ply_filenames missing.\n";
        exit (-1);
    }

    // Load old and new pointclouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
    if (pcl::io::loadPLYFile(argv[ply_filenames[0]], *p_old_pcl)) {
        std::cout << "Error loading old pointcloud!\n";
        exit(-1);
    }
    std::cout << "p_old_pcl size: " << p_old_pcl->points.size() << "\n";
    if (pcl::io::loadPLYFile(argv[ply_filenames[1]], *p_new_pcl)) {
        std::cout << "Error loading new pointcloud!\n";
        exit(-1);
    }
    std::cout << "p_new_pcl size: " << p_new_pcl->points.size() << "\n";

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_parts(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_parts(new pcl::PointCloud<pcl::PointXYZRGB>());
    CloudProjection clp_instance(p_old_pcl, p_new_pcl, p_old_parts, p_new_parts);
    clp_instance.detect_matches();

    return 0;
}
