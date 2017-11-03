#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>

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

    pcl::KdTreeFLANN<pcl::PointXYZ> kd_old_pcl;
    kd_old_pcl.setInputCloud (p_old_pcl);
    pcl::KdTreeFLANN<pcl::PointXYZ> kd_new_pcl;
    kd_new_pcl.setInputCloud (p_new_pcl);
    pcl::KdTreeFLANN<pcl::PointXYZ> kd_new_kpts;
    kd_new_kpts.setInputCloud (p_new_kpts);
    double des_radius = 0.05;
    double pos_radius = 0.03;
    int icp_iterations = 5;

    // Search most similar point from posile regions
    for (size_t i = 0; i < p_old_kpts->points.size(); ++i) {

        pcl::PointXYZ old_kpt = (p_old_kpts->points)[i];
        std::vector<int> old_neighbour_index;
        std::vector<float> old_neighbours_sqd;
        if (!kd_old_pcl.radiusSearch(old_kpt, des_radius, old_neighbour_index, old_neighbours_sqd) > 0) {
            continue;
        }
        // Old keypoint's neighbours
        pcl::PointCloud<pcl::PointXYZ>::Ptr old_neighbours(new pcl::PointCloud<pcl::PointXYZ>());
        for (size_t j = 0; j < old_neighbour_index.size(); ++j) {
            old_neighbours->points.push_back(p_old_pcl->points[old_neighbour_index[j]]);
        }

        // Get possible refer keypoints
        std::vector<int> pos_refer_index;
        std::vector<float> pos_refer_sqd;
        if (!kd_new_kpts.radiusSearch(old_kpt, pos_radius, pos_refer_index, pos_refer_sqd) > 0) {
            continue;
        }
        float best_score = 1e10;
        int best_refer_index = 0;
        for (size_t j = 0; j < pos_refer_index.size(); ++j) {
            pcl::PointXYZ new_kpt = (p_new_kpts->points)[pos_refer_index[j]];
            std::vector<int> new_neighbour_index;
            std::vector<float> new_neighbours_sqd;
            if (!kd_new_pcl.radiusSearch(new_kpt, des_radius, new_neighbour_index, new_neighbours_sqd) > 0) {
                continue;
            }
            // New keypoint's neighbours
            pcl::PointCloud<pcl::PointXYZ>::Ptr new_neighbours(new pcl::PointCloud<pcl::PointXYZ>());
            for (size_t k = 0; k < new_neighbour_index.size(); ++k) {
                new_neighbours->points.push_back(p_new_pcl->points[new_neighbour_index[k]]);
            }
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setMaximumIterations (icp_iterations);
            icp.setInputSource (new_neighbours);
            icp.setInputTarget (old_neighbours);
            icp.align (*new_neighbours);
            icp.setMaximumIterations (1);
            icp.align (*new_neighbours);
            if (icp.hasConverged()) {
                if (icp.getFitnessScore() < best_score) {
                    best_score = icp.getFitnessScore();
                    best_refer_index = j;
                    std::cout << " Matching process: " << i << "/" << p_old_kpts->points.size() <<
                    " best index: " << best_refer_index << " score: " << best_score <<
                    " refer point " << j << "/" << pos_refer_index.size()  <<
                    " size icp " << new_neighbours->points.size() <<
                    " -> " << old_neighbours->points.size() << "\n";
                }
            }
        }
    }
    return 0;
}
