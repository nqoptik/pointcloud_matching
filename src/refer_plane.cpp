#include "pointcloud_matching/cloud_diff_checker.hpp"

int main(int argc, char* argv[]) {
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    // Load the old pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr old_pcl_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPLYFile(argv[1], *old_pcl_ptr) < 0) {
        printf("Failed to load the old pointcloud!\n");
        return 1;
    }
    std::cout << "old_pcl_ptr size: " << old_pcl_ptr->points.size() << "\n";

    // Load the new pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_pcl_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPLYFile(argv[2], *new_pcl_ptr) < 0) {
        printf("Failed to load the new pointcloud!\n");
        return 1;
    }
    std::cout << "new_pcl_ptr size: " << new_pcl_ptr->points.size() << "\n";

    // Load the offset values
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

    // Shift the new pointcloud by the offset values
    for (size_t i = 0; i < new_pcl_ptr->points.size(); ++i) {
        new_pcl_ptr->points[i].x += offset_x;
        new_pcl_ptr->points[i].y += offset_y;
        new_pcl_ptr->points[i].z += offset_z;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr old_parts_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_parts_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    CloudDiffChecker CloudDiffCheckerInstance(old_pcl_ptr, new_pcl_ptr, old_parts_ptr, new_parts_ptr, argv[5]);
    std::cout << "old_parts_ptr.size: " << old_parts_ptr->points.size() << "\n";
    std::cout << "new_parts_ptr.size: " << new_parts_ptr->points.size() << "\n";
    std::ofstream ofs_pairs(argv[6]);
    ofs_pairs << std::fixed << offset1_x << " " << std::fixed << offset1_y << " " << std::fixed << offset1_z << "\n";
    for (size_t i = 0; i < old_parts_ptr->points.size(); ++i) {
        ofs_pairs << old_parts_ptr->points[i].x << " " << old_parts_ptr->points[i].y << " " << old_parts_ptr->points[i].z << " "
                  << new_parts_ptr->points[i].x << " " << new_parts_ptr->points[i].y << " " << new_parts_ptr->points[i].z << "\n";
    }
    ofs_pairs.close();
    std::cout << argv[6] << " saved.\n";
    return 0;
}
