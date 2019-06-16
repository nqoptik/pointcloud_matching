#include "pointcloud_matching/cloud_diff_checker.hpp"

CloudDiffChecker::CloudDiffChecker(const pcl::PointCloud<pcl::PointXYZ>::Ptr old_pcl_ptr,
                                   const pcl::PointCloud<pcl::PointXYZ>::Ptr new_pcl_ptr,
                                   const pcl::PointCloud<pcl::PointXYZ>::Ptr old_parts_ptr,
                                   const pcl::PointCloud<pcl::PointXYZ>::Ptr new_parts_ptr,
                                   const std::string matching_results_file) {
    old_pcl_ptr_ = old_pcl_ptr;
    new_pcl_ptr_ = new_pcl_ptr;
    old_parts_ptr_ = old_parts_ptr;
    new_parts_ptr_ = new_parts_ptr;
    matching_results_file_ = matching_results_file;
    old_diff_ptr_ = (pcl::PointCloud<pcl::PointXYZ>::Ptr)(new pcl::PointCloud<pcl::PointXYZ>());
    new_diff_ptr_ = (pcl::PointCloud<pcl::PointXYZ>::Ptr)(new pcl::PointCloud<pcl::PointXYZ>());
    old_res_ = 1.0;
    new_res_ = 1.0;
    min_moving_distance_ = 100.0f;
    x_min_ = FLT_MAX;
    y_min_ = FLT_MAX;
    z_min_ = FLT_MAX;
    x_max_ = -FLT_MAX;
    y_max_ = -FLT_MAX;
    z_max_ = -FLT_MAX;
    grid_count_ = 100;
    x_grid_count_ = 1;
    y_grid_count_ = 1;
    min_points_in_grid_ = 15;
    ransac_distance_threshold_ = 30;
    get_cloud_parameters();
    determine_diff_regions();
    gridding_diff();
    get_refer_plane();
    get_projections();
    draw_transformation();
}

CloudDiffChecker::~CloudDiffChecker() {
}

float CloudDiffChecker::distance(const pcl::PointXYZ p1, const pcl::PointXYZ p2) {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    float dz = p1.z - p2.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

float CloudDiffChecker::squared_distance(const pcl::PointXYZ p1, const pcl::PointXYZ p2) {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    float dz = p1.z - p2.z;
    return (dx * dx + dy * dy + dz * dz);
}

pcl::PointXYZ CloudDiffChecker::project_onto_plane(const pcl::PointXYZ project_point, const pcl::PointXYZ plane_point, const pcl::PointXYZ plane_normal) {
    float t_upper = plane_normal.x * (plane_point.x - project_point.x) + plane_normal.y * (plane_point.y - project_point.y) + plane_normal.z * (plane_point.z - project_point.z);
    float t_lower = plane_normal.x * plane_normal.x + plane_normal.y * plane_normal.y + plane_normal.z * plane_normal.z;
    float t = t_upper / t_lower;
    pcl::PointXYZ point;
    point.x = project_point.x + t * plane_normal.x;
    point.y = project_point.y + t * plane_normal.y;
    point.z = project_point.z + t * plane_normal.z;
    return point;
}

pcl::PointXYZ CloudDiffChecker::line_onto_plane(const pcl::PointXYZ point, const pcl::PointXYZ normal, const float a, const float b, const float c, const float d) {
    float t_upper = -(a * point.x + b * point.y + c * point.z + d);
    float t_lower = a * normal.x + b * normal.y + c * normal.z;
    float t = t_upper / t_lower;
    pcl::PointXYZ intersection;
    intersection.x = point.x + t * normal.x;
    intersection.y = point.y + t * normal.y;
    intersection.z = point.z + t * normal.z;
    return intersection;
}

double CloudDiffChecker::compute_cloud_resolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) {
    double resolution = 0.0;
    int n_points = 0;
    int n_resolution;
    std::vector<int> k_indices(2);
    std::vector<float> k_squared_distances(2);
    pcl::search::KdTree<pcl::PointXYZ> kd_tree;
    kd_tree.setInputCloud(cloud);
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (!pcl_isfinite((*cloud)[i].x)) {
            continue;
        }
        n_resolution = kd_tree.nearestKSearch(i, 2, k_indices, k_squared_distances);
        if (n_resolution == 2) {
            resolution += sqrt(k_squared_distances[1]);
            ++n_points;
        }
    }
    if (n_points != 0) {
        resolution /= n_points;
    }
    return resolution;
}

void CloudDiffChecker::determine_diff_regions() {
    old_diff_ptr_->resize(0);
    new_diff_ptr_->resize(0);
    pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree_flann_new;
    kd_tree_flann_new.setInputCloud(new_pcl_ptr_);
    for (size_t i = 0; i < old_pcl_ptr_->points.size(); ++i) {
        pcl::PointXYZ search_point = old_pcl_ptr_->points[i];
        std::vector<int> k_indices;
        std::vector<float> k_squared_distances;
        float radius = min_moving_distance_;
        if (!(kd_tree_flann_new.radiusSearch(search_point, radius, k_indices, k_squared_distances) > 0)) {
            old_diff_ptr_->points.push_back(search_point);
        }
    }
    old_diff_ptr_->width = old_diff_ptr_->points.size();
    old_diff_ptr_->height = 1;
    old_diff_ptr_->is_dense = false;
    std::cout << "Old diff size: " << old_diff_ptr_->points.size() << "\n";

    pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree_flann_old;
    kd_tree_flann_old.setInputCloud(old_pcl_ptr_);
    for (size_t i = 0; i < new_pcl_ptr_->points.size(); ++i) {
        pcl::PointXYZ search_point = new_pcl_ptr_->points[i];
        std::vector<int> k_indices;
        std::vector<float> k_squared_distances;
        float radius = min_moving_distance_;
        if (!(kd_tree_flann_old.radiusSearch(search_point, radius, k_indices, k_squared_distances) > 0)) {
            new_diff_ptr_->points.push_back(search_point);
        }
    }
    new_diff_ptr_->width = new_diff_ptr_->points.size();
    new_diff_ptr_->height = 1;
    new_diff_ptr_->is_dense = false;
    std::cout << "New diff size: " << new_diff_ptr_->points.size() << "\n";
}

void CloudDiffChecker::get_cloud_parameters() {
    old_res_ = CloudDiffChecker::compute_cloud_resolution(old_pcl_ptr_);
    new_res_ = CloudDiffChecker::compute_cloud_resolution(new_pcl_ptr_);
    std::cout << "old_res_ = " << old_res_ << "\n";
    std::cout << "new_res_ = " << new_res_ << "\n";

    for (size_t i = 0; i < old_pcl_ptr_->points.size(); ++i) {
        if (old_pcl_ptr_->points[i].x < x_min_) {
            x_min_ = old_pcl_ptr_->points[i].x;
        }
        if (old_pcl_ptr_->points[i].y < y_min_) {
            y_min_ = old_pcl_ptr_->points[i].y;
        }
        if (old_pcl_ptr_->points[i].z < z_min_) {
            z_min_ = old_pcl_ptr_->points[i].z;
        }
        if (old_pcl_ptr_->points[i].x > x_max_) {
            x_max_ = old_pcl_ptr_->points[i].x;
        }
        if (old_pcl_ptr_->points[i].y > y_max_) {
            y_max_ = old_pcl_ptr_->points[i].y;
        }
        if (old_pcl_ptr_->points[i].z > z_max_) {
            z_max_ = old_pcl_ptr_->points[i].z;
        }
    }
    for (size_t i = 0; i < new_pcl_ptr_->points.size(); ++i) {
        if (new_pcl_ptr_->points[i].x < x_min_) {
            x_min_ = new_pcl_ptr_->points[i].x;
        }
        if (new_pcl_ptr_->points[i].y < y_min_) {
            y_min_ = new_pcl_ptr_->points[i].y;
        }
        if (new_pcl_ptr_->points[i].z < z_min_) {
            z_min_ = new_pcl_ptr_->points[i].z;
        }
        if (new_pcl_ptr_->points[i].x > x_max_) {
            x_max_ = new_pcl_ptr_->points[i].x;
        }
        if (new_pcl_ptr_->points[i].y > y_max_) {
            y_max_ = new_pcl_ptr_->points[i].y;
        }
        if (new_pcl_ptr_->points[i].z > z_max_) {
            z_max_ = new_pcl_ptr_->points[i].z;
        }
    }

    std::cout << "x_min_ = " << x_min_ << " | x_max_ = " << x_max_ << " | y_min_ = " << y_min_ << " | y_max_ = "
              << y_max_ << " | z_min_ = " << z_min_ << " | z_max_ = " << z_max_ << "\n";
    d_max_ = x_max_ - x_min_;
    if (y_max_ - y_min_ > d_max_) {
        d_max_ = y_max_ - y_min_;
    }
    if (z_max_ - z_min_ > d_max_) {
        d_max_ = z_max_ - z_min_;
    }
    std::cout << "d_max_ = " << d_max_ << "\n";
    min_moving_distance_ = d_max_ / 500;
    std::cout << "min_moving_distance_ = " << min_moving_distance_ << "\n";
    float d_x = x_max_ - x_min_;
    float d_y = y_max_ - y_min_;
    if (d_x < d_y) {
        grid_step_length_ = d_x / grid_count_;
    } else {
        grid_step_length_ = d_y / grid_count_;
    }
    std::cout << "grid_step_length_ = " << grid_step_length_ << "\n";
    x_grid_count_ = ceil(d_x / grid_step_length_);
    y_grid_count_ = ceil(d_y / grid_step_length_);
    std::cout << "x_grid_count_ = " << x_grid_count_ << "\n";
    std::cout << "y_grid_count_ = " << y_grid_count_ << "\n";

    ransac_distance_threshold_ = grid_step_length_ / 10;
    std::cout << "ransac_distance_threshold_: " << ransac_distance_threshold_ << "\n";
}

void CloudDiffChecker::gridding_diff() {
    std::cout << "gridding_diff.\n";
    std::vector<std::vector<pcl::PointXYZ>> old_diff_grid_ptr(x_grid_count_ * y_grid_count_, std::vector<pcl::PointXYZ>());
    std::vector<std::vector<pcl::PointXYZ>> new_diff_grid_ptr(x_grid_count_ * y_grid_count_, std::vector<pcl::PointXYZ>());
    for (size_t i = 0; i < new_diff_ptr_->points.size(); ++i) {
        int x_grid_index = std::min((int)std::floor((new_diff_ptr_->points[i].x - x_min_) / grid_step_length_), x_grid_count_ - 1);
        int y_grid_index = std::min((int)std::floor((new_diff_ptr_->points[i].y - y_min_) / grid_step_length_), y_grid_count_ - 1);
        new_diff_grid_ptr[y_grid_index * x_grid_count_ + x_grid_index].push_back(
            pcl::PointXYZ(new_diff_ptr_->points[i].x, new_diff_ptr_->points[i].y, new_diff_ptr_->points[i].z));
    }
    for (size_t i = 0; i < old_diff_ptr_->points.size(); ++i) {
        int x_grid_index = std::min((int)std::floor((old_diff_ptr_->points[i].x - x_min_) / grid_step_length_), x_grid_count_ - 1);
        int y_grid_index = std::min((int)std::floor((old_diff_ptr_->points[i].y - y_min_) / grid_step_length_), y_grid_count_ - 1);
        old_diff_grid_ptr[y_grid_index * x_grid_count_ + x_grid_index].push_back(
            pcl::PointXYZ(old_diff_ptr_->points[i].x, old_diff_ptr_->points[i].y, old_diff_ptr_->points[i].z));
    }

    cv::Mat grid_visualization = cv::Mat::zeros(y_grid_count_, x_grid_count_, CV_8UC1);
    for (size_t i = 0; i < new_diff_grid_ptr.size(); ++i) {
        int x_index = i % x_grid_count_;
        int y_index = i / x_grid_count_;
        if (new_diff_grid_ptr[i].size() > min_points_in_grid_ &&
            old_diff_grid_ptr[i].size() > min_points_in_grid_) {
            grid_visualization.at<uchar>(y_index, x_index) = 255;
        }
    }
    std::cout << "gridding_diff done!\n";

    // Get the cluster's indices
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(grid_visualization.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    std::cout << "contours.size() = " << contours.size() << "\n";
    cv::Mat contours_img = cv::Mat(y_grid_count_, x_grid_count_, CV_8UC1, 255);
    for (size_t i = 0; i < contours.size(); ++i) {
        cv::drawContours(contours_img, contours, i, i, CV_FILLED);
    }

    cluster_indices_ = std::vector<int>(x_grid_count_ * y_grid_count_, -1);
    for (size_t i = 0; i < new_diff_grid_ptr.size(); ++i) {
        int x_index = i % x_grid_count_;
        int y_index = i / x_grid_count_;
        if ((int)contours_img.at<uchar>(y_index, x_index) == 255) {
            cluster_indices_[i] = -1;
        } else {
            cluster_indices_[i] = (int)contours_img.at<uchar>(y_index, x_index);
        }
    }

    // Index the points to fit the planes
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> old_cloud(contours.size());
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> new_cloud(contours.size());
    for (size_t i = 0; i < contours.size(); ++i) {
        old_cloud[i] = (pcl::PointCloud<pcl::PointXYZ>::Ptr)(new pcl::PointCloud<pcl::PointXYZ>());
        new_cloud[i] = (pcl::PointCloud<pcl::PointXYZ>::Ptr)(new pcl::PointCloud<pcl::PointXYZ>());
    }

    for (size_t i = 0; i < new_diff_grid_ptr.size(); ++i) {
        if (cluster_indices_[i] >= 0) {
            for (size_t j = 0; j < old_diff_grid_ptr[i].size(); ++j) {
                old_cloud[cluster_indices_[i]]->points.push_back(old_diff_grid_ptr[i][j]);
            }
            for (size_t j = 0; j < new_diff_grid_ptr[i].size(); ++j) {
                new_cloud[cluster_indices_[i]]->points.push_back(new_diff_grid_ptr[i][j]);
            }
        }
    }
    for (size_t i = 0; i < contours.size(); ++i) {
        old_cloud[i]->width = old_cloud[i]->points.size();
        old_cloud[i]->height = 1;
        old_cloud[i]->is_dense = false;
        new_cloud[i]->width = new_cloud[i]->points.size();
        new_cloud[i]->height = 1;
        new_cloud[i]->is_dense = false;
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> resized_old_cloud(contours.size());
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> resized_new_cloud(contours.size());
    for (size_t i = 0; i < contours.size(); ++i) {
        resized_old_cloud[i] = (pcl::PointCloud<pcl::PointXYZ>::Ptr)(new pcl::PointCloud<pcl::PointXYZ>());
        resized_new_cloud[i] = (pcl::PointCloud<pcl::PointXYZ>::Ptr)(new pcl::PointCloud<pcl::PointXYZ>());
    }
    for (size_t i = 0; i < new_diff_grid_ptr.size(); ++i) {
        if (cluster_indices_[i] >= 0) {
            for (size_t j = 0; j < old_diff_grid_ptr[i].size(); ++j) {
                resized_old_cloud[cluster_indices_[i]]->points.push_back(old_diff_grid_ptr[i][j]);
            }
            for (size_t j = 0; j < new_diff_grid_ptr[i].size(); ++j) {
                resized_new_cloud[cluster_indices_[i]]->points.push_back(new_diff_grid_ptr[i][j]);
            }
        }
    }

    for (size_t i = 0; i < contours.size(); ++i) {
        if (resized_old_cloud[i]->points.size() > 5000) {
            std::random_shuffle(resized_old_cloud[i]->points.begin(), resized_old_cloud[i]->points.end());
            resized_old_cloud[i]->points.resize(5000);
        }
        resized_old_cloud[i]->width = resized_old_cloud[i]->points.size();
        resized_old_cloud[i]->height = 1;
        resized_old_cloud[i]->is_dense = false;
        if (resized_new_cloud[i]->points.size() > 5000) {
            std::random_shuffle(resized_new_cloud[i]->points.begin(), resized_new_cloud[i]->points.end());
            resized_new_cloud[i]->points.resize(5000);
        }
        resized_new_cloud[i]->width = resized_new_cloud[i]->points.size();
        resized_new_cloud[i]->height = 1;
        resized_new_cloud[i]->is_dense = false;
    }

    // Fit the old planes
    plane_coefficients_ = std::vector<PlaneCoefficients>(contours.size(), PlaneCoefficients());
    for (size_t i = 0; i < contours.size(); ++i) {
        std::vector<int> inliers;
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr old_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(resized_old_cloud[i]));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(old_plane);
        ransac.setDistanceThreshold(ransac_distance_threshold_);
        ransac.computeModel();
        Eigen::VectorXf coeff;
        ransac.getModelCoefficients(coeff);
        ransac.getInliers(inliers);
        if (coeff[2] < 0) {
            for (int j = 0; j < 4; ++j) {
                coeff[j] = -coeff[j];
            }
        }
        plane_coefficients_[i].o_a = coeff[0];
        plane_coefficients_[i].o_b = coeff[1];
        plane_coefficients_[i].o_c = coeff[2];
        plane_coefficients_[i].o_d = coeff[3];
        plane_coefficients_[i].o_inliers = inliers.size();
        plane_coefficients_[i].o_total = resized_old_cloud[i]->points.size();
    }
    std::cout << "Fit old planes, done!\n";

    // Fit the new planes
    for (size_t i = 0; i < contours.size(); ++i) {
        std::vector<int> inliers;
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr new_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(resized_new_cloud[i]));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(new_plane);
        ransac.setDistanceThreshold(ransac_distance_threshold_);
        ransac.computeModel();
        Eigen::VectorXf coeff;
        ransac.getModelCoefficients(coeff);
        ransac.getInliers(inliers);
        if (coeff[2] < 0) {
            for (int j = 0; j < 4; ++j) {
                coeff[j] = -coeff[j];
            }
        }
        plane_coefficients_[i].n_a = coeff[0];
        plane_coefficients_[i].n_b = coeff[1];
        plane_coefficients_[i].n_c = coeff[2];
        plane_coefficients_[i].n_d = coeff[3];
        plane_coefficients_[i].n_inliers = inliers.size();
        plane_coefficients_[i].n_total = resized_new_cloud[i]->points.size();
    }
    std::cout << "Fit new planes, done!\n";
}

void CloudDiffChecker::get_refer_plane() {
    std::vector<ReferPlane> duplicated_refer_planes;
    duplicated_refer_planes = std::vector<ReferPlane>(x_grid_count_ * y_grid_count_, ReferPlane());
    for (size_t i = 0; i < duplicated_refer_planes.size(); ++i) {
        int x_index = i % x_grid_count_;
        int y_index = i / x_grid_count_;
        float centroid_x = x_min_ + x_index * grid_step_length_ + grid_step_length_ / 2;
        float centroid_y = y_min_ + y_index * grid_step_length_ + grid_step_length_ / 2;
        float n_centroid_z = -(plane_coefficients_[cluster_indices_[i]].n_a * centroid_x + plane_coefficients_[cluster_indices_[i]].n_b * centroid_y + plane_coefficients_[cluster_indices_[i]].n_d) / plane_coefficients_[cluster_indices_[i]].n_c;
        float o_centroid_z = -(plane_coefficients_[cluster_indices_[i]].o_a * centroid_x + plane_coefficients_[cluster_indices_[i]].o_b * centroid_y + plane_coefficients_[cluster_indices_[i]].o_d) / plane_coefficients_[cluster_indices_[i]].o_c;
        float normal_ref_x = plane_coefficients_[cluster_indices_[i]].n_a + plane_coefficients_[cluster_indices_[i]].o_a;
        float normal_ref_y = plane_coefficients_[cluster_indices_[i]].n_b + plane_coefficients_[cluster_indices_[i]].o_b;
        float normal_ref_z = plane_coefficients_[cluster_indices_[i]].n_c + plane_coefficients_[cluster_indices_[i]].o_c;
        float normal_ref_length = sqrt(normal_ref_x * normal_ref_x + normal_ref_y * normal_ref_y + normal_ref_z * normal_ref_z);
        normal_ref_x /= normal_ref_length;
        normal_ref_y /= normal_ref_length;
        normal_ref_z /= normal_ref_length;
        float centroid_z = (n_centroid_z + o_centroid_z) / 2;
        duplicated_refer_planes[i].r_a = normal_ref_x;
        duplicated_refer_planes[i].r_b = normal_ref_y;
        duplicated_refer_planes[i].r_c = normal_ref_z;
        duplicated_refer_planes[i].r_d = -(normal_ref_x * centroid_x + normal_ref_y * centroid_y + normal_ref_z * centroid_z);
    }
    refer_planes_ = std::vector<ReferPlane>(plane_coefficients_.size(), ReferPlane());
    std::vector<bool> isSetup(plane_coefficients_.size(), false);
    for (size_t i = 0; i < duplicated_refer_planes.size(); ++i) {
        int cluster_index = cluster_indices_[i];
        if (cluster_index >= 0) {
            if (!isSetup[cluster_index]) {
                refer_planes_[cluster_index].r_a = duplicated_refer_planes[i].r_a;
                refer_planes_[cluster_index].r_b = duplicated_refer_planes[i].r_b;
                refer_planes_[cluster_index].r_c = duplicated_refer_planes[i].r_c;
                refer_planes_[cluster_index].r_d = duplicated_refer_planes[i].r_d;
                isSetup[cluster_index] = true;
            }
        }
    }
    std::cout << "Get refer planes, done!\n";
}

void CloudDiffChecker::get_projections() {
    old_projection_ptr_ = (pcl::PointCloud<pcl::PointXYZ>::Ptr)(new pcl::PointCloud<pcl::PointXYZ>());
    new_projection_ptr_ = (pcl::PointCloud<pcl::PointXYZ>::Ptr)(new pcl::PointCloud<pcl::PointXYZ>());

    for (size_t i = 0; i < old_pcl_ptr_->points.size(); ++i) {
        int x_grid_index = std::min((int)std::floor((old_pcl_ptr_->points[i].x - x_min_) / grid_step_length_), x_grid_count_ - 1);
        int y_grid_index = std::min((int)std::floor((old_pcl_ptr_->points[i].y - y_min_) / grid_step_length_), y_grid_count_ - 1);
        int grid_index = y_grid_index * x_grid_count_ + x_grid_index;
        int cluster_index = cluster_indices_[grid_index];
        pcl::PointXYZ projection;
        if (cluster_index >= 0) {
            pcl::PointXYZ point = old_pcl_ptr_->points[i];
            pcl::PointXYZ normal;
            normal.x = refer_planes_[cluster_index].r_a;
            normal.y = refer_planes_[cluster_index].r_b;
            normal.z = refer_planes_[cluster_index].r_c;
            projection = line_onto_plane(point, normal, refer_planes_[cluster_index].r_a, refer_planes_[cluster_index].r_b, refer_planes_[cluster_index].r_c, refer_planes_[cluster_index].r_d);
        } else {
            projection.x = -FLT_MAX;
            projection.y = -FLT_MAX;
            projection.z = -FLT_MAX;
        }
        old_projection_ptr_->points.push_back(projection);
    }
    for (size_t i = 0; i < new_pcl_ptr_->points.size(); ++i) {
        int x_grid_index = std::min((int)std::floor((new_pcl_ptr_->points[i].x - x_min_) / grid_step_length_), x_grid_count_ - 1);
        int y_grid_index = std::min((int)std::floor((new_pcl_ptr_->points[i].y - y_min_) / grid_step_length_), y_grid_count_ - 1);
        int grid_index = y_grid_index * x_grid_count_ + x_grid_index;
        int cluster_index = cluster_indices_[grid_index];
        pcl::PointXYZ projection;
        if (cluster_index >= 0) {
            pcl::PointXYZ point = new_pcl_ptr_->points[i];
            pcl::PointXYZ normal;
            normal.x = refer_planes_[cluster_index].r_a;
            normal.y = refer_planes_[cluster_index].r_b;
            normal.z = refer_planes_[cluster_index].r_c;
            projection = line_onto_plane(point, normal, refer_planes_[cluster_index].r_a, refer_planes_[cluster_index].r_b, refer_planes_[cluster_index].r_c, refer_planes_[cluster_index].r_d);
        } else {
            projection.x = -FLT_MAX;
            projection.y = -FLT_MAX;
            projection.z = -FLT_MAX;
        }
        new_projection_ptr_->points.push_back(projection);
    }
    old_projection_ptr_->width = old_projection_ptr_->points.size();
    old_projection_ptr_->height = 1;
    old_projection_ptr_->is_dense = false;
    new_projection_ptr_->width = new_projection_ptr_->points.size();
    new_projection_ptr_->height = 1;
    new_projection_ptr_->is_dense = false;
    std::cout << "Get projections, done!\n";
}

void CloudDiffChecker::draw_transformation() {
    std::vector<pcl::PointXYZRGB> ply_scene_cloud;
    pcl::PointXYZRGB pclxyzrgb;
    pclxyzrgb.r = 0;
    pclxyzrgb.g = 0;
    pclxyzrgb.b = 255;
    for (size_t i = 0; i < old_pcl_ptr_->points.size(); ++i) {
        pclxyzrgb.x = old_pcl_ptr_->points[i].x;
        pclxyzrgb.y = old_pcl_ptr_->points[i].y;
        pclxyzrgb.z = old_pcl_ptr_->points[i].z;
        ply_scene_cloud.push_back(pclxyzrgb);
    }
    pclxyzrgb.r = 255;
    pclxyzrgb.g = 0;
    pclxyzrgb.b = 0;
    for (size_t i = 0; i < new_pcl_ptr_->points.size(); ++i) {
        pclxyzrgb.x = new_pcl_ptr_->points[i].x;
        pclxyzrgb.y = new_pcl_ptr_->points[i].y;
        pclxyzrgb.z = new_pcl_ptr_->points[i].z;
        ply_scene_cloud.push_back(pclxyzrgb);
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree_flann_old_projection;
    kd_tree_flann_old_projection.setInputCloud(old_projection_ptr_);
    pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree_flann_new_projection;
    kd_tree_flann_new_projection.setInputCloud(new_projection_ptr_);

    for (size_t i = 0; i < cluster_indices_.size(); ++i) {
        if (cluster_indices_[i] >= 0) {
            int x_index = i % x_grid_count_;
            int y_index = i / x_grid_count_;
            float centroid_x = x_min_ + x_index * grid_step_length_ + grid_step_length_ / 2;
            float centroid_y = y_min_ + y_index * grid_step_length_ + grid_step_length_ / 2;

            // Draw transform vectors
            pclxyzrgb.r = 255;
            pclxyzrgb.g = 255;
            pclxyzrgb.b = 255;
            for (float x = x_min_ + x_index * grid_step_length_; x < x_min_ + x_index * grid_step_length_ + grid_step_length_; x += grid_step_length_ / 2) {
                for (float y = y_min_ + y_index * grid_step_length_; y < y_min_ + y_index * grid_step_length_ + grid_step_length_; y += grid_step_length_ / 2) {
                    float z = -(refer_planes_[cluster_indices_[i]].r_a * x + refer_planes_[cluster_indices_[i]].r_b * y +
                                refer_planes_[cluster_indices_[i]].r_d) /
                              refer_planes_[cluster_indices_[i]].r_c;
                    pcl::PointXYZ search_point;
                    search_point.x = x;
                    search_point.y = y;
                    search_point.z = z;
                    std::vector<int> k_indices;
                    std::vector<float> k_squared_distances;
                    int min_size_proj = 0;
                    float radius = grid_step_length_ / 2;
                    pcl::PointXYZ old_terminal(0, 0, 0);
                    float old_stdv = 0;
                    if ((kd_tree_flann_old_projection.radiusSearch(search_point, radius, k_indices, k_squared_distances) > 0)) {
                        for (size_t j = 0; j < k_indices.size(); ++j) {
                            old_terminal.x += old_pcl_ptr_->points[k_indices[j]].x;
                            old_terminal.y += old_pcl_ptr_->points[k_indices[j]].y;
                            old_terminal.z += old_pcl_ptr_->points[k_indices[j]].z;
                        }
                        old_terminal.x /= k_indices.size();
                        old_terminal.y /= k_indices.size();
                        old_terminal.z /= k_indices.size();
                        for (size_t j = 0; j < k_indices.size(); ++j) {
                            pcl::PointXYZ old_point = old_pcl_ptr_->points[k_indices[j]];
                            old_stdv += squared_distance(old_terminal, old_point);
                        }
                        min_size_proj = k_indices.size();
                        old_stdv /= k_indices.size();
                        old_stdv = sqrt(old_stdv);
                    }
                    k_indices.clear();
                    k_squared_distances.clear();
                    pcl::PointXYZ new_terminal(0, 0, 0);
                    float new_stdv = 0;
                    if ((kd_tree_flann_new_projection.radiusSearch(search_point, radius, k_indices, k_squared_distances) > 0)) {
                        for (size_t j = 0; j < k_indices.size(); ++j) {
                            new_terminal.x += new_pcl_ptr_->points[k_indices[j]].x;
                            new_terminal.y += new_pcl_ptr_->points[k_indices[j]].y;
                            new_terminal.z += new_pcl_ptr_->points[k_indices[j]].z;
                        }
                        new_terminal.x /= k_indices.size();
                        new_terminal.y /= k_indices.size();
                        new_terminal.z /= k_indices.size();
                        for (size_t j = 0; j < k_indices.size(); ++j) {
                            pcl::PointXYZ new_point = new_pcl_ptr_->points[k_indices[j]];
                            new_stdv += squared_distance(new_terminal, new_point);
                        }
                        new_stdv /= k_indices.size();
                        new_stdv = sqrt(new_stdv);
                        if (k_indices.size() < min_size_proj) {
                            min_size_proj = k_indices.size();
                        }
                    }

                    if (old_terminal.x != 0 && new_terminal.x != 0 && old_stdv < d_max_ / 250 && new_stdv < d_max_ / 250 && min_size_proj > 1) {
                        old_parts_ptr_->points.push_back(old_terminal);
                        new_parts_ptr_->points.push_back(new_terminal);
                        pcl::PointXYZ vec;
                        vec.x = new_terminal.x - old_terminal.x;
                        vec.y = new_terminal.y - old_terminal.y;
                        vec.z = new_terminal.z - old_terminal.z;
                        float length = sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
                        vec.x /= length;
                        vec.y /= length;
                        vec.z /= length;
                        for (float t = 0; t < FLT_MAX; t += grid_step_length_ / 100) {
                            pclxyzrgb.x = old_terminal.x + t * vec.x;
                            pclxyzrgb.y = old_terminal.y + t * vec.y;
                            pclxyzrgb.z = old_terminal.z + t * vec.z;
                            ply_scene_cloud.push_back(pclxyzrgb);
                            if (t > length) {
                                break;
                            }
                        }
                    }
                }
            }
        }
    }
    old_parts_ptr_->width = old_parts_ptr_->points.size();
    old_parts_ptr_->height = 1;
    new_parts_ptr_->width = new_parts_ptr_->points.size();
    new_parts_ptr_->height = 1;
    std::cout << "ply_scene_cloud.size() = " << ply_scene_cloud.size() << "\n";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr trans(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (size_t i = 0; i < ply_scene_cloud.size(); ++i) {
        trans->points.push_back(ply_scene_cloud[i]);
    }
    trans->width = ply_scene_cloud.size();
    trans->height = 1;
    pcl::io::savePLYFile(matching_results_file_, *trans, true);
    std::cout << matching_results_file_ << " saved.\n";
}