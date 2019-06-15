#ifndef CLOUD_PROJECTION_HPP
#define CLOUD_PROJECTION_HPP

#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include <string>
#include <sstream>

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
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/video/tracking.hpp>

#include "pointcloud_matching/configurations.hpp"

void normalizeColours(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl);

class CloudProjection {
   private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr old_pcl_ptr_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_pcl_ptr_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr old_parts_ptr_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_parts_ptr_;
    std::vector<size_t> match_train_indices_;
    std::vector<size_t> match_query_indices_;
    std::vector<int> direction_indices_;

   public:
    CloudProjection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr old_pcl_ptr,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_pcl_ptr,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr old_parts_ptr,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_parts_ptr);
    ~CloudProjection();
    void get_matches_by_direction(Eigen::Matrix4f transform, int direction_index);
    static void get_2d_matches(cv::Mat old_project, cv::Mat new_project, double distance_threshold, std::vector<cv::Point2f>& trainPoints, std::vector<cv::Point2f>& queryPoints, int direction_index);
    void detect_matches();
};

#endif  // CLOUD_PROJECTION_HPP
