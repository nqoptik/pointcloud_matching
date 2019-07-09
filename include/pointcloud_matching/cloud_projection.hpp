#ifndef CLOUD_PROJECTION_HPP
#define CLOUD_PROJECTION_HPP

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <pcl/console/parse.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/xfeatures2d.hpp>

#include "pointcloud_matching/configurations.hpp"

void normalise_colours(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcl_ptr);

class CloudProjection
{
   private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr old_pointcloud_ptr_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_pointcloud_ptr_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr old_parts_ptr_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_parts_ptr_;
    std::vector<size_t> match_train_indices_;
    std::vector<size_t> match_query_indices_;
    std::vector<int> direction_indices_;

   public:
    CloudProjection(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_pointcloud_ptr,
                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_pointcloud_ptr,
                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_parts_ptr,
                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_parts_ptr);
    ~CloudProjection();

    void get_matches_by_direction(const Eigen::Matrix4f& transform, const int& direction_index);
    static void detect_2d_matches(const cv::Mat& old_projection_image,
                                  const cv::Mat& new_projection_image,
                                  const double& distance_threshold,
                                  std::vector<cv::Point2f>& train_points,
                                  std::vector<cv::Point2f>& query_points,
                                  const int& direction_index);
    void detect_matches();
};

#endif  // CLOUD_PROJECTION_HPP
