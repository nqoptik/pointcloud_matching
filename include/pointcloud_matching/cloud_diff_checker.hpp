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
    float o_a, o_b, o_c, o_d, o_inliers, o_total;
    float n_a, n_b, n_c, n_d, n_inliers, n_total;

    PlaneCoefficients(const float o_a = 0,
                      const float o_b = 0,
                      const float o_c = 0,
                      const float o_d = 0,
                      const float o_inliers = 0,
                      const float o_total = 0,
                      const float n_a = 0,
                      const float n_b = 0,
                      const float n_c = 0,
                      const float n_d = 0,
                      const float n_inliers = 0,
                      const float n_total = 0) {
        this->o_a = o_a;
        this->o_b = o_b;
        this->o_c = o_c;
        this->o_d = o_d;
        this->o_inliers = o_inliers;
        this->o_total = o_total;
        this->n_a = n_a;
        this->n_b = n_b;
        this->n_c = n_c;
        this->n_d = n_d;
        this->n_inliers = n_inliers;
        this->n_total = n_total;
    }
};

struct ReferPlane {
    float r_a, r_b, r_c, r_d;

    ReferPlane(const float r_a = 0, const float r_b = 0, const float r_c = 0, const float r_d = 0) {
        this->r_a = r_a;
        this->r_b = r_b;
        this->r_c = r_c;
        this->r_d = r_d;
    }
};

class CloudDiffChecker {
   private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr old_pcl_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_pcl_ptr_;
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
    double old_res_, new_res_;
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
    CloudDiffChecker(const pcl::PointCloud<pcl::PointXYZ>::Ptr old_pcl_ptr,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr new_pcl_ptr,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr old_parts_ptr,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr new_parts_ptr,
                     const std::string matching_results_file);
    ~CloudDiffChecker();

    static double compute_cloud_resolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);
    void determine_diff_regions();
    void get_cloud_parameters();
    void gridding_diff();
    void get_refer_plane();
    void get_projections();
    void draw_transformation();
    static float distance(const pcl::PointXYZ p1, const pcl::PointXYZ p2);
    static float squared_distance(const pcl::PointXYZ p1, const pcl::PointXYZ p2);
    static pcl::PointXYZ project_onto_plane(const pcl::PointXYZ project_point, const pcl::PointXYZ plane_point, const pcl::PointXYZ plane_normal);
    static pcl::PointXYZ line_onto_plane(const pcl::PointXYZ point, const pcl::PointXYZ normal, const float a, const float b, const float c, const float d);
};

#endif  // CLOUD_DIFF_CHECKER_HPP
