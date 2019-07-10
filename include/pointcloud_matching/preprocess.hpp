#ifndef PREPROCESS_HPP
#define PREPROCESS_HPP

#include <iostream>
#include <string>
#include <vector>

#include <liblas/liblas.hpp>

#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>

#include "pointcloud_matching/configurations.hpp"

#define INVALID -1
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

typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr (*FUNCTION)(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);

struct CommandOption
{
    FUNCTION noise = NULL;
    FUNCTION down_sampling = NULL;
    bool inter = false;
    char* input = NULL;
    char* offset = NULL;
    char* inter_noise = NULL;
    char* inter_down_sampling = NULL;
} command_option;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr nearest_down_sampling_with_leaf_size(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_pointcloud_ptr,
                                                                            const double& leaf_size);

// The noise filtering functions
pcl::PointCloud<pcl::PointXYZRGB>::Ptr stat_filtering_noise(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_pointcloud_ptr);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr radius_filtering_noise(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_pointcloud_ptr);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_based_filtering_noise(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_pointcloud_ptr);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr stat_color_filtering_noise(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_pointcloud_ptr);

// The down sampling functions
pcl::PointCloud<pcl::PointXYZRGB>::Ptr median_down_sampling(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_pointcloud_ptr);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr nearest_down_sampling(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_pointcloud_ptr);

// The array reference function
FUNCTION functions[] = {
    stat_filtering_noise,
    radius_filtering_noise,
    color_based_filtering_noise,
    stat_color_filtering_noise,
    median_down_sampling,
    nearest_down_sampling};

const char* method_name[] = {
    // The noise filtering methods
    "stat",
    "radius",
    "colorbased",
    "statcolor",

    // The down sampling methods
    "median",
    "nearestmed"};

const char* options[] = {
    // The noise filtering option
    "-n",

    // The down sampling option
    "-d",

    // The intermediate option
    "-inter",

    // The input file option
    "-i",

    // The output file option
    "-ofs",

    // The intermediate noise option
    "-in",

    // The intermediate downsampling option
    "-ids"};

#endif // PREPROCESS_HPP
