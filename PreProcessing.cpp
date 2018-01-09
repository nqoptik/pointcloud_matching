#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include <string>

#include <liblas/liblas.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "Configurations.h"

void normalizeColours(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl) {

    double r_avg = 0, g_avg = 0, b_avg = 0;
    for (size_t i = 0; i < p_pcl->points.size(); ++i) {
        r_avg += p_pcl->points[i].r;
        g_avg += p_pcl->points[i].g;
        b_avg += p_pcl->points[i].b;
    }
    r_avg /= p_pcl->points.size();
    g_avg /= p_pcl->points.size();
    b_avg /= p_pcl->points.size();

    // Compute colours' standard deviation
    double r_std = 0, g_std = 0, b_std = 0;
    for (size_t i = 0; i < p_pcl->points.size(); ++i) {
        r_std += (r_avg - p_pcl->points[i].r)*(r_avg - p_pcl->points[i].r);
        g_std += (g_avg - p_pcl->points[i].g)*(g_avg - p_pcl->points[i].g);
        b_std += (b_avg - p_pcl->points[i].b)*(b_avg - p_pcl->points[i].b);
    }
    r_std /= p_pcl->points.size();
    g_std /= p_pcl->points.size();
    b_std /= p_pcl->points.size();
    r_std = sqrt(r_std);
    g_std = sqrt(g_std);
    b_std = sqrt(b_std);

    // Synchronize pointclouds' colours
    for (size_t i = 0; i < p_pcl->points.size(); ++i) {
        p_pcl->points[i].r = std::max(0.0, std::min(255.0, (100.0 + (p_pcl->points[i].r - r_avg) * 35.0 / r_std)));
        p_pcl->points[i].g = std::max(0.0, std::min(255.0, (100.0 + (p_pcl->points[i].g - g_avg) * 35.0 / g_std)));
        p_pcl->points[i].b = std::max(0.0, std::min(255.0, (100.0 + (p_pcl->points[i].b - b_avg) * 35.0 / b_std)));
    }
}

int main (int argc, char** argv) {

    Configurations::getInstance()->readConfig();

    std::vector<int> las_filenames;
    las_filenames = pcl::console::parse_file_extension_argument (argc, argv, ".las");
    if (las_filenames.size () != 1)
    {
        std::cout << "las_filenames missing.\n";
        exit (-1);
    }
    std::vector<int> ply_filenames;
    ply_filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");
    if (ply_filenames.size () != 1)
    {
        std::cout << "ply_filenames missing.\n";
        exit (-1);
    }
    std::string las_file = argv[las_filenames[0]];
    std::string ply_file = argv[ply_filenames[0]];

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_orgCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    std::ifstream ifs;
    ifs.open(las_file.c_str(), std::ios::in | std::ios::binary);
    liblas::ReaderFactory f;
    liblas::Reader reader = f.CreateWithStream(ifs);
    liblas::Header const& header = reader.GetHeader();
    std::cout << "las_file size: " << header.GetPointRecordsCount() << "\n";

    double offset_x = Configurations::getInstance()->offset_x;
    double offset_y = Configurations::getInstance()->offset_y;
    double offset_z = Configurations::getInstance()->offset_z;
    while (reader.ReadNextPoint()) {

        liblas::Point const& p = reader.GetPoint();
        pcl::PointXYZRGB cur_point;
        cur_point.x = p.GetX() - offset_x;
        cur_point.y = p.GetY() - offset_y;
        cur_point.z = p.GetZ() - offset_z;
        cur_point.b = p.GetColor().GetBlue()/256;
        cur_point.g = p.GetColor().GetGreen()/256;
        cur_point.r = p.GetColor().GetRed()/256;
        p_orgCloud->points.push_back(cur_point);
    }
    std::cout << "ply_file : " << *p_orgCloud << "\n";

    // Down sampling
    pcl::VoxelGrid<pcl::PointXYZRGB> grid;
    double leaf = Configurations::getInstance()->leaf_size;
    grid.setLeafSize(leaf, leaf, leaf);
    grid.setInputCloud(p_orgCloud);
    grid.filter(*p_orgCloud);
    std::cout << "ply_file after down sampling: " << *p_orgCloud << "\n";
    normalizeColours(p_orgCloud);

    // Create the filtering object
    for (int loop = 0; loop < 15; ++loop) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud (p_orgCloud);
        sor.setMeanK (Configurations::getInstance()->sor_neighbours);
        sor.setStddevMulThresh (Configurations::getInstance()->sor_stdev_thresh);
        sor.filter (*p_orgCloud);
        int cur_size = p_orgCloud->points.size();
        std::cout << "Loop: " << loop << " ply_file after filtering: " << cur_size << "\n";
    }

    pcl::io::savePLYFile(ply_file, *p_orgCloud, true);
    std::cout << ply_file << " saved\n";
    return 0;
}
