#ifndef _PREPROCESS_H_
#define _PREPROCESS_H_

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

#define UNVALID -1
#define ERROR  "There is a problem when parsing command option"
#define HELP "\nUSAGE: " \
                    "\t-n <nose_filtering>\n " \
                        "\t\tstat\n" \
                        "\t\tradius\n" \
                        "\t\tcolorbased\n" \
                        "\t\tbilateral\n" \
                    "\t-d <down_sampleling>\n" \
                        "\t\tvirtual\n" \
                        "\t\treal\n" \
                    "\t-inter <is_intermediate>\n" \
                        "\t\tY/y or Yes/yes\n" \
                        "\t\tN/n or No/no\n" \
                    "\t-i <input_file>\n" \
                    "\t-o <output_file>\n"

typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr (*FUNCTION)(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
struct CommandOption {
    FUNCTION noise = NULL;
    FUNCTION down_sample = NULL;
    bool inter = false;
    char* input = NULL;
    char* output = NULL;
} commandOption;

/* noise filtering methods*/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr statFilteringNoise(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr radiusFilteringNoise(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorbasedFilteringNoise(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr bilateralFilteringNoise(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);

/* Down Sampling */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr downSamplingVirtualCenter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr downSamplingRealCenter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);


// array reference to function
FUNCTION functions[] = {
            statFilteringNoise,
            radiusFilteringNoise,
            colorbasedFilteringNoise,
            bilateralFilteringNoise,
            downSamplingVirtualCenter,
            downSamplingRealCenter
};

const char* methodName[] = {
            /* noise filter */
            "stat",
            "radius",
            "colorbased",
            "bilateral",

            /* down sampling */
            "virtual",
            "real"
};

const char* options[] = {
            // noise filtering
            "-n",

            // down sampling
            "-d",

            // intermediate
            "-inter", // defaule is no id not exists

            // input file
            "-i",

            // output file
            "-o"
};

#endif