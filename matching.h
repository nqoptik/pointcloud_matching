#pragma once

#ifndef _MATCHING_H_
#define _MATCHING_H_

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

#include "Configurations.h"
#include "CloudProjection.h"
#include "CloudDiffChecker.h"

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

void drawKeypoints(std::string fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
void drawMatchingResults(std::string, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr);

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
void issDetectKeypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, bool);
void susanDetectKeypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, bool);
void harris3dDetectkeypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, bool);

/* descriptor detect method */
void twoDimensionDetectKeypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
void icpDetectDescriptor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
void shotDetectDescriptor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
void fpfhDetectDescriptor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
void shotcolorDetectDescriptor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr);

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

#endif
