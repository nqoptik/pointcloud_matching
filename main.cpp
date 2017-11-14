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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
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

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_parts(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_parts(new pcl::PointCloud<pcl::PointXYZRGB>());

    for (size_t octave = 0; octave < 1; ++octave) {

        // Detect ISS keypints
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_kpts(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss_old_detector;
        iss_old_detector.setSalientRadius(Configurations::getInstance()->iss_old_salient_radius);
        iss_old_detector.setNonMaxRadius(Configurations::getInstance()->iss_old_nonmax_radius);
        iss_old_detector.setMinNeighbors(Configurations::getInstance()->iss_old_min_neighbours);
        iss_old_detector.setInputCloud(p_old_pcl);
        iss_old_detector.compute(*p_old_kpts);
        std::cout << "p_old_kpts " << *p_old_kpts << "\n";

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_kpts(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss_new_detector;
        iss_new_detector.setSalientRadius(Configurations::getInstance()->iss_new_salient_radius);
        iss_new_detector.setNonMaxRadius(Configurations::getInstance()->iss_new_nonmax_radius);
        iss_new_detector.setMinNeighbors(Configurations::getInstance()->iss_new_min_neighbours);
        iss_new_detector.setInputCloud(p_new_pcl);
        iss_new_detector.compute(*p_new_kpts);
        std::cout << "p_new_kpts " << *p_new_kpts << "\n";

        pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_old_pcl;
        kd_old_pcl.setInputCloud (p_old_pcl);
        pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_new_pcl;
        kd_new_pcl.setInputCloud (p_new_pcl);
        pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_new_kpts;
        kd_new_kpts.setInputCloud (p_new_kpts);
        double des_radius = Configurations::getInstance()->des_radius;
        double pos_radius = Configurations::getInstance()->pos_radius;
        int icp_iterations = Configurations::getInstance()->icp_iterations;
        double refine_radius = Configurations::getInstance()->refine_radius;

        // Search most similar point from possile regions
        for (size_t i = 0; i < p_old_kpts->points.size(); ++i) {

            std::cout << " Matching process: " << i << "/" << p_old_kpts->points.size() << "\n";
            pcl::PointXYZRGB old_kpt = (p_old_kpts->points)[i];
            std::vector<int> old_neighbour_index;
            std::vector<float> old_neighbours_sqd;
            if (!kd_old_pcl.radiusSearch(old_kpt, des_radius, old_neighbour_index, old_neighbours_sqd) > 0) {
                std::cout << " !not found old neighbours\n";
                continue;
            }
            // Old keypoint's neighbours
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr old_neighbours(new pcl::PointCloud<pcl::PointXYZRGB>());
            for (size_t j = 0; j < old_neighbour_index.size(); ++j) {
                old_neighbours->points.push_back(p_old_pcl->points[old_neighbour_index[j]]);
            }

            // Get possible refer keypoints
            std::vector<int> pos_refer_index;
            std::vector<float> pos_refer_sqd;
            if (!kd_new_kpts.radiusSearch(old_kpt, pos_radius, pos_refer_index, pos_refer_sqd) > 0) {
                std::cout << " !not found possible refer keypoint.\n";
                continue;
            }
            float best_score = 1e10;
            int best_refer_index = 0;
            for (size_t j = 0; j < pos_refer_index.size(); ++j) {
                pcl::PointXYZRGB new_kpt = (p_new_kpts->points)[pos_refer_index[j]];
                std::vector<int> new_neighbour_index;
                std::vector<float> new_neighbours_sqd;
                if (!kd_new_pcl.radiusSearch(new_kpt, des_radius, new_neighbour_index, new_neighbours_sqd) > 0) {
                    std::cout << " !not found new neighbours\n";
                    continue;
                }
                // New keypoint's neighbours
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_neighbours(new pcl::PointCloud<pcl::PointXYZRGB>());
                for (size_t k = 0; k < new_neighbour_index.size(); ++k) {
                    new_neighbours->points.push_back(p_new_pcl->points[new_neighbour_index[k]]);
                }
                pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
                icp.setMaximumIterations(icp_iterations);
                icp.setInputSource(new_neighbours);
                icp.setInputTarget(old_neighbours);
                icp.align(*new_neighbours);
                icp.setMaximumIterations(1);
                icp.align(*new_neighbours);
                if (icp.hasConverged()) {
                    if (icp.getFitnessScore() < best_score) {
                        best_score = icp.getFitnessScore();
                        best_refer_index = j;
                        std::cout << " best index: " << best_refer_index << " score: " << best_score <<
                        " refer point " << j << "/" << pos_refer_index.size()  <<
                        " size icp " << new_neighbours->points.size() <<
                        " -> " << old_neighbours->points.size() << "\n";
                    }
                }
            }
            if (best_score < 1) {
                std::cout << "Refine";
                // Get refine points
                pcl::PointXYZRGB similar_kpt = p_new_kpts->points[pos_refer_index[best_refer_index]];
                std::vector<int> refine_index;
                std::vector<float> refine_sqd;
                if (!kd_new_pcl.radiusSearch(similar_kpt, refine_radius, refine_index, refine_sqd) > 0) {
                    std::cout << " !not found refine point\n";
                    continue;
                }
                float refine_best_score = 1e10;
                int refine_best_refer_index = 0;
                for (size_t j = 0; j < refine_index.size(); ++j) {
                    pcl::PointXYZRGB new_similar_point = (p_new_pcl->points)[refine_index[j]];
                    std::vector<int> new_neighbour_index;
                    std::vector<float> new_neighbours_sqd;
                    if (!kd_new_pcl.radiusSearch(new_similar_point, des_radius, new_neighbour_index, new_neighbours_sqd) > 0) {
                        std::cout << " !not found new neighbours\n";
                        continue;
                    }
                    // New keypoint's neighbours
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_neighbours(new pcl::PointCloud<pcl::PointXYZRGB>());
                    for (size_t k = 0; k < new_neighbour_index.size(); ++k) {
                        new_neighbours->points.push_back(p_new_pcl->points[new_neighbour_index[k]]);
                    }
                    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
                    icp.setMaximumIterations(icp_iterations);
                    icp.setInputSource(new_neighbours);
                    icp.setInputTarget(old_neighbours);
                    icp.align(*new_neighbours);
                    icp.setMaximumIterations(1);
                    icp.align(*new_neighbours);
                    if (icp.hasConverged()) {
                        if (icp.getFitnessScore() < refine_best_score) {
                            refine_best_score = icp.getFitnessScore();
                            refine_best_refer_index = j;
                            std::cout << " precess: " << j << " / " << refine_index.size() << "\n";
                        }
                    }
                }
                pcl::PointXYZRGB nearestPoint = p_new_pcl->points[refine_index[refine_best_refer_index]];
                p_old_parts->points.push_back(old_kpt);
                p_new_parts->points.push_back(nearestPoint);
            }
        }
    }

    // Draw matches;
    std::vector<pcl::PointXYZRGB> ply_matches;
    for (size_t i = 0; i < p_old_pcl->points.size(); ++i) {
        pcl::PointXYZRGB tmp;
        if (Configurations::getInstance()->draw_old_colour) {
            tmp.r = p_old_pcl->points[i].r;
            tmp.g = p_old_pcl->points[i].g;
            tmp.b = p_old_pcl->points[i].b;
        }
        else {
            tmp.r = 0;
            tmp.g = 0;
            tmp.b = 255;
        }
        tmp.x = p_old_pcl->points[i].x;
        tmp.y = p_old_pcl->points[i].y;
        tmp.z = p_old_pcl->points[i].z;
        ply_matches.push_back(tmp);
    }
    for (size_t i = 0; i < p_new_pcl->points.size(); ++i) {
        pcl::PointXYZRGB tmp;
        if (Configurations::getInstance()->draw_new_colour) {
            tmp.r = p_new_pcl->points[i].r;
            tmp.g = p_new_pcl->points[i].g;
            tmp.b = p_new_pcl->points[i].b;
        }
        else {
            tmp.r = 255;
            tmp.g = 0;
            tmp.b = 0;
        }
        tmp.x = p_new_pcl->points[i].x;
        tmp.y = p_new_pcl->points[i].y;
        tmp.z = p_new_pcl->points[i].z;
        ply_matches.push_back(tmp);
    }
    for (size_t i = 0; i < p_old_parts->points.size(); ++i) {
        pcl::PointXYZRGB tmp;
        tmp.r = 255;
        tmp.g = 255;
        tmp.b = 255;
        pcl::PointXYZRGB vec;
        vec.x = p_new_parts->points[i].x - p_old_parts->points[i].x;
        vec.y = p_new_parts->points[i].y - p_old_parts->points[i].y;
        vec.z = p_new_parts->points[i].z - p_old_parts->points[i].z;
        float length = sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
        vec.x /= length;
        vec.y /= length;
        vec.z /= length;
        for (float t = 0; t < 1e10; t += Configurations::getInstance()->leaf_size / 100)
        {
            if (t > length) {
                break;
            }
            tmp.x = p_old_parts->points[i].x + t * vec.x;
            tmp.y = p_old_parts->points[i].y + t * vec.y;
            tmp.z = p_old_parts->points[i].z + t * vec.z;
            ply_matches.push_back(tmp);
        }
    }
    savePly("Matches.ply", ply_matches);
    std::cout << "Matches saved.\n";
    return 0;
}
