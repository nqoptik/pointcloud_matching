#pragma once

#ifndef _CLOUDPROJECTION_H_
#define _CLOUDPROJECTION_H_


#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/video/tracking.hpp>   

#include "Configurations.h"

class CloudProjection {

private:

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_pcl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_pcl;
    std::vector<size_t> match_train_indices;
    std::vector<size_t> match_query_indices;
    std::vector<int> direction_indices;
    static void normalizeColours(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl);
    void get_matches_by_direction(Eigen::Matrix4f transform, int direction_index);
    static void get_2d_matches(cv::Mat old_project, cv::Mat new_project, std::vector<cv::Point2f>& trainPoints,
        std::vector<cv::Point2f>& queryPoints);
    void draw_matches();

public:

    CloudProjection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_pcl,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_pcl);
    ~CloudProjection();
    void detect_matches();
};

#endif /* _CLOUDPROJECTION_H_ */