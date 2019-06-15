#ifndef PREPROCESS_HPP
#define PREPROCESS_HPP

#include <iostream>
#include <string>
#include <vector>

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

#include "pointcloud_matching/configurations.hpp"

#define UNVALID -1
#define ERROR "There is a problem when parsing command option"
#define HELP                       \
    "\nUSAGE: "                    \
    "\t-n <nose_filtering>\n "     \
    "\t\tstat\n"                   \
    "\t\tradius\n"                 \
    "\t\tcolorbased\n"             \
    "\t\tbilateral\n"              \
    "\t-d <down_sampleling>\n"     \
    "\t\tvirtual\n"                \
    "\t\treal\n"                   \
    "\t-inter <is_intermediate>\n" \
    "\t\tY/y or Yes/yes\n"         \
    "\t\tN/n or No/no\n"           \
    "\t-i <input_file>\n"          \
    "\t-ofs <offset_file>\n"

typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr (*FUNCTION)(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
struct CommandOption {
    FUNCTION noise = NULL;
    FUNCTION down_sample = NULL;
    bool inter = false;
    char* input = NULL;
    char* offset = NULL;
    char* interNoise = NULL;
    char* interDownSample = NULL;
} commandOption;

/* noise filtering methods*/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr stat_filtering_noise(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr radius_filtering_noise(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorbased_filtering_noise(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr statcolor_filtering_noise(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);

/* Down Sampling */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr median_down_sampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr nearest_median_down_sampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);

// array reference to function
FUNCTION functions[] = {
    stat_filtering_noise,
    radius_filtering_noise,
    colorbased_filtering_noise,
    statcolor_filtering_noise,
    median_down_sampling,
    nearest_median_down_sampling};

const char* methodName[] = {
    /* noise filter */
    "stat",
    "radius",
    "colorbased",
    "statcolor",

    /* down sampling */
    "median",
    "nearestmed"};

const char* options[] = {
    // noise filtering
    "-n",

    // down sampling
    "-d",

    // intermediate
    "-inter",  // default is no id not exists

    // input file
    "-i",

    // output file
    "-ofs",

    //char* interNoise = NULL;
    "-in",

    //char* interDownSample = NULL;
    "-ids"};

#endif  // PREPROCESS_HPP
