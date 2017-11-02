#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include <string>

#include <liblas/liblas.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/kdtree/kdtree_flann.h>

int main (int argc, char** argv) {

    std::vector<int> las_filenames;
    las_filenames = pcl::console::parse_file_extension_argument (argc, argv, ".las");
    if (las_filenames.size () != 1)
    {
        std::cout << "las_filenames missing.\n";
        exit (-1);
    }
    std::vector<int> pcd_filenames;
    pcd_filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
    if (pcd_filenames.size () != 1)
    {
        std::cout << "pcd_filenames missing.\n";
        exit (-1);
    }
    std::string las_file = argv[las_filenames[0]];
    std::string pcd_file = argv[pcd_filenames[0]];

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_orgCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    std::ifstream ifs;
    ifs.open(las_file.c_str(), std::ios::in | std::ios::binary);
    liblas::ReaderFactory f;
    liblas::Reader reader = f.CreateWithStream(ifs);
    liblas::Header const& header = reader.GetHeader();
    std::cout << "las_file size: " << header.GetPointRecordsCount() << "\n";

    while (reader.ReadNextPoint()) {

        liblas::Point const& p = reader.GetPoint();
        pcl::PointXYZRGB cur_point;
        cur_point.x = p.GetX();
        cur_point.y = p.GetY();
        cur_point.z = p.GetZ();
        cur_point.b = p.GetColor().GetBlue()/256;
        cur_point.g = p.GetColor().GetGreen()/256;
        cur_point.r = p.GetColor().GetRed()/256;
        p_orgCloud->points.push_back(cur_point);
    }
    std::cout << "pcd_file : " << *p_orgCloud << "\n";

    // Down sampling
    pcl::VoxelGrid<pcl::PointXYZRGB> grid;
    double leaf = 0.0015;
    grid.setLeafSize(leaf, leaf, leaf);
    grid.setInputCloud(p_orgCloud);
    grid.filter(*p_orgCloud);
    std::cout << "pcd_file after down sampling: " << *p_orgCloud << "\n";

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (p_orgCloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (4.0);
    sor.filter (*p_orgCloud);
    std::cout << "pcd_file after filtering: " << *p_orgCloud << "\n";

    pcl::io::savePCDFileASCII(pcd_file, *p_orgCloud);
    std::cout << pcd_file << " saved\n";
    return 0;
}
