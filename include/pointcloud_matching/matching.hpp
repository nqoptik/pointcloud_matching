#ifndef MATCHING_HPP
#define MATCHING_HPP

#define PCL_NO_PRECOMPILE

#include <iostream>
#include <string>
#include <vector>

#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/susan.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/icp.h>

#include "pointcloud_matching/configurations.hpp"
#include "pointcloud_matching/cloud_projection.hpp"
#include "pointcloud_matching/cloud_diff_checker.hpp"

#define UNVALID -1
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

void draw_keypoints(std::string fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
void draw_matching_results(std::string, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr);

union FUNCTION {
    void (*f3)(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, bool);

    void (*f6)(pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
               pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
               pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
               pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
               pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
               pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
};

struct CommandOption {
    FUNCTION keypoint_detect_method;
    FUNCTION descriptor_detect_methos;
    bool inter = false;
    char* input1 = NULL;
    char* input2 = NULL;
    char* output = NULL;
    char* interKeypoint1 = NULL;
    char* interKeypoint2 = NULL;
    char* matchingPairs = NULL;
    char* offset1 = NULL;
    char* offset2 = NULL;
} commandOption;

/* keypoints detect method */
void iss_detect_keypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, bool);
void susan_detect_keypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, bool);
void harris3d_detect_keypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, bool);

/* descriptor detect method */
void two_dimension_detect_keypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
void icp_extract_description(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
void shot_extract_description(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
void fpfh_extract_description(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
void shotcolor_extract_description(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr);

const char* methodName[] = {
    /* keypoint detect method */
    "iss",
    "susan",
    "harris3D",

    /* descriptor detect method */
    "2dmethod",
    "icp",
    "shot",
    "fpfh",
    "shotcolor"};

const char* options[] = {
    // noise filtering
    "-kp",

    // down sampling
    "-des",

    // intermediate, defaule is no id not exists
    "-inter",

    // input file
    "-i1",
    "-i2",

    // output file
    "-o",

    // output keypoints 1
    "-ikp1",

    // output keypoints 2
    "-ikp2",

    // matching pairs
    "-mp",

    // offset 1
    "-ofs1",

    // offset 2
    "-ofs2"};

#endif  // MATCHING_HPP