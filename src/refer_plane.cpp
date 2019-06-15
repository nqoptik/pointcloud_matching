#include "pointcloud_matching/cloud_diff_checker.hpp"

int main(int argc, char* argv[]) {
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    /* Load old and new pointclouds */
    pcl::PointCloud<pcl::PointXYZ>::Ptr p_old_pcl(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr p_new_pcl(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPLYFile(argv[1], *p_old_pcl) < 0) {
        std::cout << "Error loading old pointcloud!\n";
        return 1;
    }
    std::cout << "p_old_pcl size: " << (*p_old_pcl).points.size() << "\n";

    if (pcl::io::loadPLYFile(argv[2], *p_new_pcl) < 0) {
        std::cout << "Error loading new pointcloud!\n";
        return 1;
    }
    std::cout << "p_new_pcl size: " << (*p_new_pcl).points.size() << "\n";
    double offset1_x, offset1_y, offset1_z;
    double offset2_x, offset2_y, offset2_z;
    double offset_x, offset_y, offset_z;
    std::ifstream ifs_ofs1(argv[3]);
    ifs_ofs1 >> std::fixed >> offset1_x;
    ifs_ofs1 >> std::fixed >> offset1_y;
    ifs_ofs1 >> std::fixed >> offset1_z;
    ifs_ofs1.close();
    std::ifstream ifs_ofs2(argv[4]);
    ifs_ofs2 >> std::fixed >> offset2_x;
    ifs_ofs2 >> std::fixed >> offset2_y;
    ifs_ofs2 >> std::fixed >> offset2_z;
    ifs_ofs2.close();
    offset_x = offset2_x - offset1_x;
    offset_y = offset2_y - offset1_y;
    offset_z = offset2_z - offset1_z;
    for (size_t i = 0; i < p_new_pcl->points.size(); ++i) {
        p_new_pcl->points[i].x += offset_x;
        p_new_pcl->points[i].y += offset_y;
        p_new_pcl->points[i].z += offset_z;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr p_old_parts(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr p_new_parts(new pcl::PointCloud<pcl::PointXYZ>());
    CloudDiffChecker CloudDiffCheckerInstance(p_old_pcl, p_new_pcl, p_old_parts, p_new_parts, argv[5]);
    std::cout << "p_old_parts.size: " << p_old_parts->points.size() << "\n";
    std::cout << "p_new_parts.size: " << p_new_parts->points.size() << "\n";
    std::ofstream ofs_pairs(argv[6]);
    ofs_pairs << std::fixed << offset1_x << " " << std::fixed << offset1_y << " " << std::fixed << offset1_z << "\n";
    for (size_t i = 0; i < p_old_parts->points.size(); ++i) {
        ofs_pairs << p_old_parts->points[i].x << " " << p_old_parts->points[i].y << " " << p_old_parts->points[i].z << " " << p_new_parts->points[i].x << " " << p_new_parts->points[i].y << " " << p_new_parts->points[i].z << "\n";
    }
    ofs_pairs.close();
    std::cout << argv[6] << " saved.\n";
    return 0;
}
