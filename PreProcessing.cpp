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
#include <pcl/filters/radius_outlier_removal.h>
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
    p_orgCloud->width = p_orgCloud->points.size();
    p_orgCloud->height = 1;
    std::cout << "ply_file : " << *p_orgCloud << "\n";
    std::cout << "X: " << header.GetMinX() << " -> " << header.GetMaxX()  << "\n";
    std::cout << "Y: " << header.GetMinY() << " -> " << header.GetMaxY()  << "\n";
    std::cout << "Z: " << header.GetMinZ() << " -> " << header.GetMaxZ()  << "\n";
    std::cout << "gerchar.\n";
    getchar();

    // Down sampling
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_ds_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::VoxelGrid<pcl::PointXYZRGB> grid;
    double leaf = Configurations::getInstance()->leaf_size;
    grid.setLeafSize(leaf, leaf, leaf);
    grid.setInputCloud(p_orgCloud);
    grid.filter(*p_ds_pcl);
    std::cout << "ply_file after down sampling: " << *p_ds_pcl << "\n";

    // Down sampling with real point near center
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_org_pcl;
    kd_org_pcl.setInputCloud(p_orgCloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_ds_near_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (size_t i = 0; i < p_ds_pcl->points.size(); ++i) {
        pcl::PointXYZRGB centerP = p_ds_pcl->points[i];
        std::vector<int> pointIdxNKNSearch;
        std::vector<float> pointNKNSquaredDistance;
        if (kd_org_pcl.nearestKSearch (centerP, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            if (pointNKNSquaredDistance[0] < leaf*leaf) {
                p_ds_near_pcl->points.push_back(p_orgCloud->points[pointIdxNKNSearch[0]]);
            }
        }
    }
    if (p_ds_near_pcl->points.size() != p_ds_pcl->points.size()) {
        std::cout << "Need to check down sampling with real piont center.\n";
        return 1;
    }
    p_ds_near_pcl->width = p_ds_near_pcl->points.size();
    p_ds_near_pcl->height = 1;
    pcl::io::savePLYFile(ply_file, *p_ds_near_pcl, true);
    std::cout << ply_file << " saved\n";
    std::cout << "Down down sampling with real point near center.\n";
    std::cout << "ply_file after down sampling: " << *p_ds_near_pcl << "\n";


    // Radius outliers removal
    for (int loop = 0; loop < 5; ++loop) {
        pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
        ror.setInputCloud(p_ds_near_pcl);
        ror.setRadiusSearch(Configurations::getInstance()->leaf_size*3);
        ror.setMinNeighborsInRadius (10);
        // apply filter
        ror.filter (*p_ds_near_pcl);
        std::cout << "Loop: " << loop << " ply_file after applied radius outlier romoval: " << p_ds_near_pcl->points.size() << "\n";
    }

    // // Create the filtering object
    // for (int loop = 0; loop < 1; ++loop) {
    //     pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    //     sor.setInputCloud (p_ds_near_pcl);
    //     sor.setMeanK (Configurations::getInstance()->sor_neighbours);
    //     sor.setStddevMulThresh (Configurations::getInstance()->sor_stdev_thresh);
    //     sor.filter (*p_ds_near_pcl);
    //     std::cout << "Loop: " << loop << " ply_file after filtering: " << p_ds_near_pcl->points.size() << "\n";
    // }
    pcl::io::savePLYFile(ply_file, *p_ds_near_pcl, true);
    std::cout << ply_file << " saved\n";

    // Color-based filtering
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_ds_color_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_near_pcl;
    kd_near_pcl.setInputCloud(p_ds_near_pcl);
    for (size_t i = 0; i < p_ds_near_pcl->points.size(); ++i) {
        pcl::PointXYZRGB nearP = p_ds_near_pcl->points[i];
        int K = Configurations::getInstance()->sor_neighbours;
        std::vector<int> pointIdxNKNSearch;
        std::vector<float> pointNKNSquaredDistance;
        if (kd_near_pcl.nearestKSearch (nearP, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            pcl::PointXYZRGB colorP = nearP;
            float avg_r = 0;
            float avg_g = 0;
            float avg_b = 0;
            for (size_t j = 0; j < K; ++j) {
                avg_r += p_ds_near_pcl->points[pointIdxNKNSearch[j]].r;
                avg_g += p_ds_near_pcl->points[pointIdxNKNSearch[j]].g;
                avg_b += p_ds_near_pcl->points[pointIdxNKNSearch[j]].b;
            }
            avg_r /= K;
            avg_g /= K;
            avg_b /= K;
            float std_dev = 0;
            for (size_t j = 0; j < K; ++j) {
                float dr = p_ds_near_pcl->points[pointIdxNKNSearch[j]].r - avg_r;
                float dg = p_ds_near_pcl->points[pointIdxNKNSearch[j]].g - avg_g;
                float db = p_ds_near_pcl->points[pointIdxNKNSearch[j]].b - avg_b;
                float d_color = dr*dr + dg*dg + db*db;
                std_dev += d_color;
            }
            std_dev = sqrt(std_dev);
            std_dev /= K;
            float dr_color = colorP.r - avg_r;
            float dg_color = colorP.g - avg_g;
            float db_color = colorP.b - avg_b;
            float d_colorP = sqrt(dr_color*dr_color + dg_color*dg_color + db_color*db_color);
            if (d_colorP/std_dev < 10) {
                p_ds_color_pcl->points.push_back(colorP);
            }
            else {
            }
        }
    }
    p_ds_color_pcl->width = p_ds_color_pcl->points.size();
    p_ds_color_pcl->height = 1;
    std::cout << "ply_file after color-based filtering: " << *p_ds_color_pcl << "\n";

    pcl::io::savePLYFile(ply_file, *p_ds_color_pcl, true);
    std::cout << ply_file << " saved\n";
    return 0;
}
