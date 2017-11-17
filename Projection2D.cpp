#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include <string>

#include <pcl/io/pcd_io.h>
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

#include "plyio.h"
#include "Configurations.h"

int main (int argc, char** argv) {

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
    double r_avg_old = 0, g_avg_old = 0, b_avg_old = 0;
    double r_avg_new = 0, g_avg_new = 0, b_avg_new = 0;
    for (size_t i = 0; i < p_old_pcl->points.size(); ++i) {
        r_avg_old += p_old_pcl->points[i].r;
        g_avg_old += p_old_pcl->points[i].g;
        b_avg_old += p_old_pcl->points[i].b;
    }
    r_avg_old /= p_old_pcl->points.size();
    g_avg_old /= p_old_pcl->points.size();
    b_avg_old /= p_old_pcl->points.size();

    for (size_t i = 0; i < p_new_pcl->points.size(); ++i) {
        r_avg_new += p_new_pcl->points[i].r;
        g_avg_new += p_new_pcl->points[i].g;
        b_avg_new += p_new_pcl->points[i].b;
    }
    r_avg_new /= p_new_pcl->points.size();
    g_avg_new /= p_new_pcl->points.size();
    b_avg_new /= p_new_pcl->points.size();
    std::cout << "r: " << r_avg_old << "->" << r_avg_new << "\n";
    std::cout << "g: " << g_avg_old << "->" << g_avg_new << "\n";
    std::cout << "b: " << b_avg_old << "->" << b_avg_new << "\n";
    double r_std_old = 0, g_std_old = 0, b_std_old = 0;
    double r_std_new = 0, g_std_new = 0, b_std_new = 0;
    for (size_t i = 0; i < p_old_pcl->points.size(); ++i) {
        r_std_old += (r_avg_old - p_old_pcl->points[i].r)*(r_avg_old - p_old_pcl->points[i].r);
        g_std_old += (g_avg_old - p_old_pcl->points[i].g)*(g_avg_old - p_old_pcl->points[i].g);
        b_std_old += (b_avg_old - p_old_pcl->points[i].b)*(b_avg_old - p_old_pcl->points[i].b);
    }
    r_std_old /= p_old_pcl->points.size();
    g_std_old /= p_old_pcl->points.size();
    b_std_old /= p_old_pcl->points.size();
    r_std_old = sqrt(r_std_old);
    g_std_old = sqrt(g_std_old);
    b_std_old = sqrt(b_std_old);
    for (size_t i = 0; i < p_new_pcl->points.size(); ++i) {
        r_std_new += (r_avg_new - p_new_pcl->points[i].r)*(r_avg_new - p_new_pcl->points[i].r);
        g_std_new += (g_avg_new - p_new_pcl->points[i].g)*(g_avg_new - p_new_pcl->points[i].g);
        b_std_new += (b_avg_new - p_new_pcl->points[i].b)*(b_avg_new - p_new_pcl->points[i].b);
    }
    r_std_new /= p_new_pcl->points.size();
    g_std_new /= p_new_pcl->points.size();
    b_std_new /= p_new_pcl->points.size();
    r_std_new = sqrt(r_std_new);
    g_std_new = sqrt(g_std_new);
    b_std_new = sqrt(b_std_new);
    std::cout << "std r: " << r_std_old << "->" << r_std_new << "\n";
    std::cout << "std g: " << g_std_old << "->" << g_std_new << "\n";
    std::cout << "std b: " << b_std_old << "->" << b_std_new << "\n";
    for (size_t i = 0; i < p_old_pcl->points.size(); ++i) {
        p_old_pcl->points[i].r = std::max(0.0, std::min(255.0, (100.0 + (p_old_pcl->points[i].r - r_avg_old) * 35.0 / r_std_old)));
        p_old_pcl->points[i].g = std::max(0.0, std::min(255.0, (100.0 + (p_old_pcl->points[i].g - g_avg_old) * 35.0 / g_std_old)));
        p_old_pcl->points[i].b = std::max(0.0, std::min(255.0, (100.0 + (p_old_pcl->points[i].b - b_avg_old) * 35.0 / b_std_old)));
    }
    for (size_t i = 0; i < p_new_pcl->points.size(); ++i) {
        p_new_pcl->points[i].r = std::max(0.0, std::min(255.0, (100.0 + (p_new_pcl->points[i].r - r_avg_new) * 35.0 / r_std_new)));
        p_new_pcl->points[i].g = std::max(0.0, std::min(255.0, (100.0 + (p_new_pcl->points[i].g - g_avg_new) * 35.0 / g_std_new)));
        p_new_pcl->points[i].b = std::max(0.0, std::min(255.0, (100.0 + (p_new_pcl->points[i].b - b_avg_new) * 35.0 / b_std_new)));
    }
    pcl::io::savePLYFile("p_old_pcl.ply", *p_old_pcl, true);
    pcl::io::savePLYFile("p_new_pcl.ply", *p_new_pcl, true);
    std::cout << "Save with new colours\n";
    double x_min = 1e10, y_min = 1e10, z_min = 1e10;
    double x_max = -1e10, y_max = -1e10, z_max = -1e10;
    for (size_t i = 0; i < p_old_pcl->points.size(); ++i) {
        if (p_old_pcl->points[i].x < x_min) x_min = p_old_pcl->points[i].x;
        if (p_old_pcl->points[i].y < y_min) y_min = p_old_pcl->points[i].y;
        if (p_old_pcl->points[i].z < z_min) z_min = p_old_pcl->points[i].z;
        if (p_old_pcl->points[i].x > x_max) x_max = p_old_pcl->points[i].x;
        if (p_old_pcl->points[i].y > y_max) y_max = p_old_pcl->points[i].y;
        if (p_old_pcl->points[i].z > z_max) z_max = p_old_pcl->points[i].z;
    }
    for (size_t i = 0; i < p_new_pcl->points.size(); ++i) {
        if (p_new_pcl->points[i].x < x_min) x_min = p_new_pcl->points[i].x;
        if (p_new_pcl->points[i].y < y_min) y_min = p_new_pcl->points[i].y;
        if (p_new_pcl->points[i].z < z_min) z_min = p_new_pcl->points[i].z;
        if (p_new_pcl->points[i].x > x_max) x_max = p_new_pcl->points[i].x;
        if (p_new_pcl->points[i].y > y_max) y_max = p_new_pcl->points[i].y;
        if (p_new_pcl->points[i].z > z_max) z_max = p_new_pcl->points[i].z;
    }
    std::cout << "x: " << x_min << " ~ " << x_max << "\n";
    std::cout << "y: " << y_min << " ~ " << y_max << "\n";
    std::cout << "z: " << z_min << " ~ " << z_max << "\n";
    double dx = x_max - x_min;
    double dy = y_max - y_min;
    double d_min = std::min(dx, dy);
    std::cout << "d_min: " << d_min << "\n";
    double distance_threshold = d_min / 700;
    int x_size = floor(dx/distance_threshold) + 1;
    int y_size = floor(dy/distance_threshold) + 1;
    std::cout << "image size: " << x_size << "x" << y_size << "\n";

    cv::Mat zeroMat_old = cv::Mat::zeros(y_size, x_size, CV_8UC3);
    cv::Mat zMat_old(y_size, x_size, CV_32FC1, -1e10);
    for (size_t i = 0; i < p_old_pcl->points.size(); ++i) {
        int x_idx = floor((p_old_pcl->points[i].x - x_min)/distance_threshold);
        int y_idx = floor((p_old_pcl->points[i].y - y_min)/distance_threshold);
        if (p_old_pcl->points[i].z > zMat_old.at<float>(y_idx, x_idx)) {
            zeroMat_old.at<cv::Vec3b>(y_idx, x_idx)[0] = p_old_pcl->points[i].b;
            zeroMat_old.at<cv::Vec3b>(y_idx, x_idx)[1] = p_old_pcl->points[i].g;
            zeroMat_old.at<cv::Vec3b>(y_idx, x_idx)[2] = p_old_pcl->points[i].r;
            zMat_old.at<float>(y_idx, x_idx) = p_old_pcl->points[i].z;
        }
    }
    cv::flip(zeroMat_old.clone(), zeroMat_old, 0);
    cv::imshow("zeroMat_old", zeroMat_old);
    cv::imwrite("zeroMat_old.png", zeroMat_old);
    cv::Mat zeroMat_new = cv::Mat::zeros(y_size, x_size, CV_8UC3);
    cv::Mat zMat_new(y_size, x_size, CV_32FC1, -1e10);
    for (size_t i = 0; i < p_new_pcl->points.size(); ++i) {
        int x_idx = floor((p_new_pcl->points[i].x - x_min)/distance_threshold);
        int y_idx = floor((p_new_pcl->points[i].y - y_min)/distance_threshold);
        if (p_new_pcl->points[i].z > zMat_new.at<float>(y_idx, x_idx)) {
            zeroMat_new.at<cv::Vec3b>(y_idx, x_idx)[0] = p_new_pcl->points[i].b;
            zeroMat_new.at<cv::Vec3b>(y_idx, x_idx)[1] = p_new_pcl->points[i].g;
            zeroMat_new.at<cv::Vec3b>(y_idx, x_idx)[2] = p_new_pcl->points[i].r;
            zMat_new.at<float>(y_idx, x_idx) = p_new_pcl->points[i].z;
        }
    }
    cv::flip(zeroMat_new.clone(), zeroMat_new, 0);
    cv::imshow("zeroMat_new", zeroMat_new);
    cv::imwrite("zeroMat_new.png", zeroMat_new);
    cv::waitKey();
    return 0;
}