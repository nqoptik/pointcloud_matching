#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/keypoints/iss_3d.h>

#include "plyio.h"
#include "Configurations.h"

int main (int argc, char** argv)
{

    Configurations::getInstance()->readConfig();
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    std::vector<int> pcd_filenames;
    pcd_filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
    if (pcd_filenames.size () != 2)
    {
        std::cout << "pcd_filenames missing.\n";
        exit (-1);
    }

    // Load old and new pointclouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr p_old_pcl(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr p_new_pcl(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile(argv[pcd_filenames[0]], *p_old_pcl)) {
        std::cout << "Error loading old pointcloud!\n";
        exit(-1);
    }
    std::cout << "p_old_pcl " << *p_old_pcl << "\n";
    if (pcl::io::loadPCDFile(argv[pcd_filenames[1]], *p_new_pcl)) {
        std::cout << "Error loading new pointcloud!\n";
        exit(-1);
    }
    std::cout << "p_new_pcl " << *p_new_pcl << "\n";

    // Detect ISS keypints
    pcl::PointCloud<pcl::PointXYZ>::Ptr p_old_kpts(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_old_detector;
    iss_old_detector.setSalientRadius(Configurations::getInstance()->iss_old_salient_radius);
    iss_old_detector.setNonMaxRadius(Configurations::getInstance()->iss_old_nonmax_radius);
    iss_old_detector.setMinNeighbors(Configurations::getInstance()->iss_old_min_neighbours);
    iss_old_detector.setInputCloud(p_old_pcl);
    iss_old_detector.compute(*p_old_kpts);
    std::cout << "p_old_kpts " << *p_old_kpts << "\n";

    pcl::PointCloud<pcl::PointXYZ>::Ptr p_new_kpts(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_new_detector;
    iss_new_detector.setSalientRadius(Configurations::getInstance()->iss_new_salient_radius);
    iss_new_detector.setNonMaxRadius(Configurations::getInstance()->iss_new_nonmax_radius);
    iss_new_detector.setMinNeighbors(Configurations::getInstance()->iss_new_min_neighbours);
    iss_new_detector.setInputCloud(p_new_pcl);
    iss_new_detector.compute(*p_new_kpts);
    std::cout << "p_new_kpts " << *p_new_kpts << "\n";
    return 0;
}
