#ifndef CLOUD_DIFF_CHECKER_HPP
#define CLOUD_DIFF_CHECKER_HPP

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <string>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml/ml.hpp>

struct PlaneCoefficients {
    float old_a, old_b, old_c, old_d, old_inliers, old_total;
    float new_a, new_b, new_c, new_d, new_inliers, new_total;

    PlaneCoefficients(const float old_a = 0,
                      const float old_b = 0,
                      const float old_c = 0,
                      const float old_d = 0,
                      const float old_inliers = 0,
                      const float old_total = 0,
                      const float new_a = 0,
                      const float new_b = 0,
                      const float new_c = 0,
                      const float new_d = 0,
                      const float new_inliers = 0,
                      const float new_total = 0) {
        this->old_a = old_a;
        this->old_b = old_b;
        this->old_c = old_c;
        this->old_d = old_d;
        this->old_inliers = old_inliers;
        this->old_total = old_total;
        this->new_a = new_a;
        this->new_b = new_b;
        this->new_c = new_c;
        this->new_d = new_d;
        this->new_inliers = new_inliers;
        this->new_total = new_total;
    }
};

struct ReferPlane {
    float refer_a, refer_b, refer_c, refer_d;

    ReferPlane(const float refer_a = 0, const float refer_b = 0, const float refer_c = 0, const float refer_d = 0) {
        this->refer_a = refer_a;
        this->refer_b = refer_b;
        this->refer_c = refer_c;
        this->refer_d = refer_d;
    }
};

class CloudDiffChecker {
   private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr old_pointcloud_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_pointcloud_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr old_parts_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_parts_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr old_diff_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_diff_ptr_;
    std::vector<PlaneCoefficients> plane_coefficients_;
    std::vector<int> cluster_indices_;
    std::vector<ReferPlane> refer_planes_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr old_projection_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_projection_ptr_;
    std::string matching_results_file_;
    double old_resolution_, new_resolution_;
    float min_moving_distance_;
    float x_min_, x_max_;
    float y_min_, y_max_;
    float z_min_, z_max_;
    float d_max_;
    int grid_count_;
    int min_points_in_grid_;
    int x_grid_count_, y_grid_count_;
    float grid_step_length_;
    float ransac_distance_threshold_;

   public:
    CloudDiffChecker(const pcl::PointCloud<pcl::PointXYZ>::Ptr old_pointcloud_ptr,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr new_pointcloud_ptr,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr old_parts_ptr,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr new_parts_ptr,
                     const std::string matching_results_file);
    ~CloudDiffChecker();

    static double compute_pointcloud_resolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pointcloud_ptr);
    void determine_diff_regions();
    void get_pointcloud_parameters();
    void gridding_diff();
    void get_refer_plane();
    void get_projections();
    void draw_transformation();
    static float distance(const pcl::PointXYZ point_1, const pcl::PointXYZ point_2);
    static float squared_distance(const pcl::PointXYZ point_1, const pcl::PointXYZ point_2);
    static pcl::PointXYZ project_onto_plane(const pcl::PointXYZ project_point, const pcl::PointXYZ plane_point, const pcl::PointXYZ plane_normal);
    static pcl::PointXYZ line_onto_plane(const pcl::PointXYZ point_xyz, const pcl::PointXYZ normal, const float a, const float b, const float c, const float d);
};

#endif  // CLOUD_DIFF_CHECKER_HPP
