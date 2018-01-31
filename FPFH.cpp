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
#include <pcl/features/fpfh_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>

#include "Configurations.h"

int main (int argc, char** argv) {

    Configurations::getInstance()->readConfig();
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    std::vector<int> ply_filenames;
    ply_filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");
    if (ply_filenames.size() != 2) {
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
    std::cout << "p_old_pcl " << *p_old_pcl << "\n";
    if (pcl::io::loadPLYFile(argv[ply_filenames[1]], *p_new_pcl)) {
        std::cout << "Error loading new pointcloud!\n";
        exit(-1);
    }
    std::cout << "p_new_pcl " << *p_new_pcl << "\n";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_parts(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_parts(new pcl::PointCloud<pcl::PointXYZRGB>());

    for (size_t octave = 0; octave < 1; ++octave) {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_pcl_in(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_pcl_in(new pcl::PointCloud<pcl::PointXYZRGB>());
        double leaf_size;
        int sor_neighbours;
        double iss_old_salient_radius;
        double iss_old_nonmax_radius;
        int iss_old_min_neighbours = Configurations::getInstance()->iss_old_min_neighbours;;
        double iss_new_salient_radius;
        double iss_new_nonmax_radius;
        int iss_new_min_neighbours = Configurations::getInstance()->iss_new_min_neighbours;;
        double fpfh_radius;
        double pos_radius;
        int icp_iterations = Configurations::getInstance()->icp_iterations;
        if (octave == 0) {
            leaf_size = Configurations::getInstance()->leaf_size;
            iss_old_salient_radius = Configurations::getInstance()->iss_old_salient_radius;
            iss_old_nonmax_radius = Configurations::getInstance()->iss_old_nonmax_radius;
            iss_new_salient_radius = Configurations::getInstance()->iss_new_salient_radius;
            iss_new_nonmax_radius = Configurations::getInstance()->iss_new_nonmax_radius;
            fpfh_radius = Configurations::getInstance()->fpfh_radius;
            pos_radius = Configurations::getInstance()->pos_radius;

        }
        else if (octave == 1) {
            leaf_size = Configurations::getInstance()->leaf_size*2;
            iss_old_salient_radius = Configurations::getInstance()->iss_old_salient_radius;
            iss_old_nonmax_radius = Configurations::getInstance()->iss_old_nonmax_radius;
            iss_new_salient_radius = Configurations::getInstance()->iss_new_salient_radius*2;
            iss_new_nonmax_radius = Configurations::getInstance()->iss_new_nonmax_radius*2;
            fpfh_radius = Configurations::getInstance()->fpfh_radius*2;
            pos_radius = Configurations::getInstance()->pos_radius*2;
        }

        // Down sampling
        pcl::VoxelGrid<pcl::PointXYZRGB> grid;
        grid.setLeafSize(leaf_size, leaf_size, leaf_size);
        grid.setInputCloud(p_old_pcl);
        grid.filter(*p_old_pcl_in);
        std::cout << "p_old_pcl_in after down sampling: " << *p_old_pcl_in << "\n";
        grid.setInputCloud(p_new_pcl);
        grid.filter(*p_new_pcl_in);
        std::cout << "p_new_pcl_in after down sampling: " << *p_new_pcl_in << "\n";

        pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_old_pcl;
        kd_old_pcl.setInputCloud (p_old_pcl_in);
        pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_new_pcl;
        kd_new_pcl.setInputCloud (p_new_pcl_in);

        // Detect ISS keypints
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_kpts(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss_old_detector;
        iss_old_detector.setSalientRadius(iss_new_salient_radius);
        iss_old_detector.setNonMaxRadius(iss_new_nonmax_radius);
        iss_old_detector.setMinNeighbors(iss_new_min_neighbours);
        iss_old_detector.setInputCloud(p_old_pcl_in);
        iss_old_detector.compute(*p_old_kpts);
        std::cout << "p_old_kpts " << *p_old_kpts << "\n";
        boost::shared_ptr<std::vector<int> > old_kpt_idx_in_pcl(new std::vector<int>(0));
        for (size_t i = 0; i < p_old_kpts->points.size(); ++i) {
            pcl::PointXYZRGB old_point = p_old_kpts->points[i];
            std::vector<int> old_neighbour_index;
            std::vector<float> old_neighbours_sqd;
            if (!kd_old_pcl.nearestKSearch(old_point, 1, old_neighbour_index, old_neighbours_sqd) > 0) {
                std::cout << " Check get old keypoint's indices.\n";
                exit(1);
            }
            old_kpt_idx_in_pcl->push_back(old_neighbour_index[0]);
        }
        std::cout << "Done get old keypoint's indices.\n";

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_kpts(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss_new_detector;
        iss_new_detector.setSalientRadius(iss_new_salient_radius);
        iss_new_detector.setNonMaxRadius(iss_new_nonmax_radius);
        iss_new_detector.setMinNeighbors(iss_new_min_neighbours);
        iss_new_detector.setInputCloud(p_new_pcl_in);
        iss_new_detector.compute(*p_new_kpts);
        std::cout << "p_new_kpts " << *p_new_kpts << "\n";
        boost::shared_ptr<std::vector<int> > new_kpt_idx_in_pcl(new std::vector<int>(0));
        for (size_t i = 0; i < p_new_kpts->points.size(); ++i) {
            pcl::PointXYZRGB new_point = p_new_kpts->points[i];
            std::vector<int> new_neighbour_index;
            std::vector<float> new_neighbours_sqd;
            if (!kd_old_pcl.nearestKSearch(new_point, 1, new_neighbour_index, new_neighbours_sqd) > 0) {
                std::cout << " Check get new keypoint's indices.\n";
                exit(1);
            }
            new_kpt_idx_in_pcl->push_back(new_neighbour_index[0]);
        }
        std::cout << "Done get new keypoint's indices.\n";

        // Compute FPFH's descriptors
        pcl::PointCloud<pcl::Normal>::Ptr p_old_normal_in(new pcl::PointCloud<pcl::Normal>());
        pcl::PointCloud<pcl::Normal>::Ptr p_new_normal_in(new pcl::PointCloud<pcl::Normal>());
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
        norm_est.setSearchMethod(tree);
        norm_est.setKSearch(10);
        std::cout << "Compute old pointcloud's normals.\n";
        norm_est.setInputCloud(p_old_pcl_in);
        norm_est.compute(*p_old_normal_in);
        std::cout << "Compute new pointcloud's normals.\n";
        norm_est.setInputCloud(p_new_pcl_in);
        norm_est.compute(*p_new_normal_in);

        std::cout << "Compute old pointcloud's fpfh descriptors.\n";
        pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh_old;
        fpfh_old.setInputCloud(p_old_pcl_in);
        fpfh_old.setIndices(old_kpt_idx_in_pcl);
        fpfh_old.setInputNormals(p_old_normal_in);
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr old_fpfhs(new pcl::PointCloud<pcl::FPFHSignature33> ());
        fpfh_old.setRadiusSearch(Configurations::getInstance()->fpfh_radius);
        fpfh_old.compute (*old_fpfhs);
        std::cout << "old_fpfhs " << *old_fpfhs << "\n";

        std::cout << "Compute new pointcloud's fpfh descriptors.\n";
        pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh_new;
        fpfh_new.setInputCloud(p_new_pcl_in);
        fpfh_new.setIndices(new_kpt_idx_in_pcl);
        fpfh_new.setInputNormals(p_new_normal_in);
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr new_fpfhs(new pcl::PointCloud<pcl::FPFHSignature33> ());
        fpfh_new.setRadiusSearch(Configurations::getInstance()->fpfh_radius);
        fpfh_new.compute (*new_fpfhs);
        std::cout << "new_fpfhs " << *new_fpfhs << "\n";

        pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_new_kpts;
        kd_new_kpts.setInputCloud (p_new_kpts);
        for (size_t i = 0; i < p_old_kpts->points.size(); ++i) {
            std::cout << "Step " << i << " / " << p_old_kpts->points.size() << "\n";
            pcl::PointXYZRGB old_kpt = p_old_kpts->points[i];
            // Get possible refer keypoints
            std::vector<int> pos_refer_index;
            std::vector<float> pos_refer_sqd;
            if (!kd_new_kpts.radiusSearch(old_kpt, pos_radius, pos_refer_index, pos_refer_sqd) > 0) {
                std::cout << " !not found possible refer keypoint.\n";
                continue;
            }
            int old_fpfh_idx = i;
            int new_fpfh_idx = -1;
            float min_fpfh_d = 13.5;
            for (size_t j = 0; j < pos_refer_index.size(); ++j) {
                int new_fpfh_idx_ = pos_refer_index[j];
                float fpfh_d = 0;
                for (int k = 0; k < 33; ++k) {
                    fpfh_d += (old_fpfhs->points[old_fpfh_idx].histogram[k] - new_fpfhs->points[new_fpfh_idx_].histogram[k]) * 
                        (old_fpfhs->points[old_fpfh_idx].histogram[k] - new_fpfhs->points[new_fpfh_idx_].histogram[k]);
                }
                fpfh_d = sqrt(fpfh_d);
                if (fpfh_d < min_fpfh_d) {
                    min_fpfh_d = fpfh_d;
                    new_fpfh_idx = new_fpfh_idx_;
                }
            }
            pcl::PointXYZRGB old_part = old_kpt;
            pcl::PointXYZRGB new_part = p_new_kpts->points[new_fpfh_idx];
            if (new_fpfh_idx != -1) {
                p_old_parts->points.push_back(old_part);
                p_new_parts->points.push_back(new_part);
            }
        }
    }

    // Draw matches;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_matches(new pcl::PointCloud<pcl::PointXYZRGB>());
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
        p_matches->points.push_back(tmp);
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
        p_matches->points.push_back(tmp);
    }
    for (size_t i = 0; i < p_old_parts->points.size(); ++i) {
        pcl::PointXYZRGB tmp;
        tmp.r = 0;
        tmp.g = 255;
        tmp.b = 0;
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
            p_matches->points.push_back(tmp);
        }
    }
    p_matches->width = p_matches->points.size();
    p_matches->height = 1;
    p_matches->is_dense = 1;
    pcl::io::savePLYFile("Matches.ply", *p_matches, true);
    std::cout << "Matches saved.\n";
    return 0;
}