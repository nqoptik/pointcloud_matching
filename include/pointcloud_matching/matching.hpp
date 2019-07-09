#ifndef MATCHING_HPP
#define MATCHING_HPP

#define PCL_NO_PRECOMPILE

#include <iostream>
#include <string>
#include <vector>

#include <pcl/console/parse.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/susan.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>

#include "pointcloud_matching/cloud_diff_checker.hpp"
#include "pointcloud_matching/cloud_projection.hpp"
#include "pointcloud_matching/configurations.hpp"

#define INVALID -1
#define ERROR "There is a problem when parsing command option"
#define HELP                                          \
    "\nUSAGE: \n"                                     \
    "\t-kp <option> : keypoints detect method. \n"    \
    "\t\tiss\n"                                       \
    "\t\tsusan\n"                                     \
    "\t\tharris3D\n"                                  \
    "\t\t2dmethod\n"                                  \
    "\t-des<descriptor>: descriptor detect method.\n" \
    "\t\ticp\n"                                       \
    "\t\tshot\n"                                      \
    "\t\tfpfh\n"                                      \
    "\t-inter <is_intermediate>\n"                    \
    "\t\tY/y or Yes/yes\n"                            \
    "\t\tN/n or No/no\n"                              \
    "\t-i1 :  .ply file\n"                            \
    "\t-i2 : .ply file\n"                             \
    "\t-o : output file\n"

void draw_keypoints(const std::string& file_name,
                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud_ptr,
                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& keypoints_ptr);
void draw_matching_results(const std::string& file_name,
                           const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_pointcloud_ptr,
                           const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_pointcloud_ptr,
                           const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_parts_ptr,
                           const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_parts_ptr);

union FUNCTION {
    void (*f3)(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, const bool&);

    void (*f6)(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&,
               const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&,
               const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&,
               const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&,
               const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&,
               const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);
};

struct CommandOption
{
    FUNCTION keypoint_detect_method;
    FUNCTION descriptor_detect_methos;
    bool inter = false;
    char* input1 = NULL;
    char* input2 = NULL;
    char* output = NULL;
    char* inter_keypoint_1 = NULL;
    char* inter_keypoint_2 = NULL;
    char* matching_pairs = NULL;
    char* offset1 = NULL;
    char* offset2 = NULL;
} command_option;

// The keypoints detection functions
void iss_detect_keypoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud_ptr,
                          const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& keypoints_ptr,
                          const bool& is_before);
void susan_detect_keypoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud_ptr,
                            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& keypoints_ptr,
                            const bool& is_before);
void harris3d_detect_keypoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud_ptr,
                               const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& keypoints_ptr,
                               const bool& is_before);

void two_dimension_detect_keypoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr old_pointcloud_ptr,
                                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_pointcloud_ptr,
                                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr old_keypoints_ptr,
                                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_keypoints_ptr,
                                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr old_parts_ptr,
                                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_parts_ptr);

// The descriptors extraction functions
void icp_extract_description(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_pointcloud_ptr,
                             const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_pointcloud_ptr,
                             const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_keypoints_ptr,
                             const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_keypoints_ptr,
                             const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_parts_ptr,
                             const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_parts_ptr);
void shot_extract_description(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_pointcloud_ptr,
                              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_pointcloud_ptr,
                              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_keypoints_ptr,
                              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_keypoints_ptr,
                              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_parts_ptr,
                              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_parts_ptr);
void fpfh_extract_description(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_pointcloud_ptr,
                              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_pointcloud_ptr,
                              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_keypoints_ptr,
                              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_keypoints_ptr,
                              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_parts_ptr,
                              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_parts_ptr);
void shotcolor_extract_description(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_pointcloud_ptr,
                                   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_pointcloud_ptr,
                                   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_keypoints_ptr,
                                   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_keypoints_ptr,
                                   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_parts_ptr,
                                   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_parts_ptr);

const char* method_name[] = {
    // The keypoint detection methods
    "iss",
    "susan",
    "harris3D",

    // The desctiptor extraction methods
    "2dmethod",
    "icp",
    "shot",
    "fpfh",
    "shotcolor"};

const char* options[] = {
    // The keypoint detecion option
    "-kp",

    // The descriptor extraction option
    "-des",

    // The intermediate option
    "-inter",

    // The input files
    "-i1",
    "-i2",

    // The output file
    "-o",

    // The output keypoints 1
    "-ikp1",

    // The output keypoints 2
    "-ikp2",

    // The matching pairs
    "-mp",

    // The offset value 1
    "-ofs1",

    // The offset value 2
    "-ofs2"};

#endif  // MATCHING_HPP
