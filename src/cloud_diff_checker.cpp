#include "pointcloud_matching/cloud_diff_checker.hpp"

CloudDiffChecker::CloudDiffChecker(const pcl::PointCloud<pcl::PointXYZ>::Ptr& old_pointcloud_ptr,
                                   const pcl::PointCloud<pcl::PointXYZ>::Ptr& new_pointcloud_ptr,
                                   const pcl::PointCloud<pcl::PointXYZ>::Ptr& old_parts_ptr,
                                   const pcl::PointCloud<pcl::PointXYZ>::Ptr& new_parts_ptr,
                                   const std::string& matching_results_file)
    : old_pointcloud_ptr_(old_pointcloud_ptr),
      new_pointcloud_ptr_(new_pointcloud_ptr),
      old_parts_ptr_(old_parts_ptr),
      new_parts_ptr_(new_parts_ptr),
      matching_results_file_(matching_results_file)
{
    old_diff_ptr_ = (pcl::PointCloud<pcl::PointXYZ>::Ptr)(new pcl::PointCloud<pcl::PointXYZ>());
    new_diff_ptr_ = (pcl::PointCloud<pcl::PointXYZ>::Ptr)(new pcl::PointCloud<pcl::PointXYZ>());
    old_resolution_ = 1.0;
    new_resolution_ = 1.0;
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
    get_pointcloud_parameters();
    determine_diff_regions();
    gridding_diff();
    get_refer_plane();
    get_projections();
    draw_transformation();
}

CloudDiffChecker::~CloudDiffChecker()
{
}

float CloudDiffChecker::distance(const pcl::PointXYZ& point_1, const pcl::PointXYZ& point_2)
{
    float d_x = point_1.x - point_2.x;
    float d_y = point_1.y - point_2.y;
    float d_z = point_1.z - point_2.z;
    return sqrt(d_x * d_x + d_y * d_y + d_z * d_z);
}

float CloudDiffChecker::squared_distance(const pcl::PointXYZ& point_1, const pcl::PointXYZ& point_2)
{
    float d_x = point_1.x - point_2.x;
    float d_y = point_1.y - point_2.y;
    float d_z = point_1.z - point_2.z;
    return (d_x * d_x + d_y * d_y + d_z * d_z);
}

pcl::PointXYZ CloudDiffChecker::project_onto_plane(const pcl::PointXYZ& project_point, const pcl::PointXYZ& plane_point, const pcl::PointXYZ& plane_normal)
{
    float t_upper = plane_normal.x * (plane_point.x - project_point.x) + plane_normal.y * (plane_point.y - project_point.y) + plane_normal.z * (plane_point.z - project_point.z);
    float t_lower = plane_normal.x * plane_normal.x + plane_normal.y * plane_normal.y + plane_normal.z * plane_normal.z;
    float t = t_upper / t_lower;
    pcl::PointXYZ point_xyz;
    point_xyz.x = project_point.x + t * plane_normal.x;
    point_xyz.y = project_point.y + t * plane_normal.y;
    point_xyz.z = project_point.z + t * plane_normal.z;
    return point_xyz;
}

pcl::PointXYZ CloudDiffChecker::line_onto_plane(const pcl::PointXYZ& point_xyz, const pcl::PointXYZ& normal, const float& a, const float& b, const float& c, const float& d)
{
    float t_upper = -(a * point_xyz.x + b * point_xyz.y + c * point_xyz.z + d);
    float t_lower = a * normal.x + b * normal.y + c * normal.z;
    float t = t_upper / t_lower;
    pcl::PointXYZ intersection;
    intersection.x = point_xyz.x + t * normal.x;
    intersection.y = point_xyz.y + t * normal.y;
    intersection.z = point_xyz.z + t * normal.z;
    return intersection;
}

double CloudDiffChecker::compute_pointcloud_resolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pointcloud_ptr)
{
    double resolution = 0.0;
    int points_count = 0;
    std::vector<int> k_indices(2);
    std::vector<float> k_squared_distances(2);
    pcl::search::KdTree<pcl::PointXYZ> kd_tree;
    kd_tree.setInputCloud(pointcloud_ptr);
    for (size_t i = 0; i < pointcloud_ptr->size(); ++i)
    {
        if (!pcl_isfinite((*pointcloud_ptr)[i].x))
        {
            continue;
        }
        int nearest_points_count = kd_tree.nearestKSearch(i, 2, k_indices, k_squared_distances);
        if (nearest_points_count == 2)
        {
            resolution += sqrt(k_squared_distances[1]);
            ++points_count;
        }
    }
    if (points_count != 0)
    {
        resolution /= points_count;
    }
    return resolution;
}

void CloudDiffChecker::determine_diff_regions()
{
    old_diff_ptr_->resize(0);
    new_diff_ptr_->resize(0);

    // Initialise the KdTreeFLANN for the new pointcloud
    pcl::KdTreeFLANN<pcl::PointXYZ> new_kd_tree_flann;
    new_kd_tree_flann.setInputCloud(new_pointcloud_ptr_);
    for (size_t i = 0; i < old_pointcloud_ptr_->points.size(); ++i)
    {
        pcl::PointXYZ search_point = old_pointcloud_ptr_->points[i];
        std::vector<int> k_indices;
        std::vector<float> k_squared_distances;
        float radius = min_moving_distance_;
        if (!(new_kd_tree_flann.radiusSearch(search_point, radius, k_indices, k_squared_distances) > 0))
        {
            old_diff_ptr_->points.push_back(search_point);
        }
    }
    old_diff_ptr_->width = old_diff_ptr_->points.size();
    old_diff_ptr_->height = 1;
    old_diff_ptr_->is_dense = false;
    std::cout << "Old diff size: " << old_diff_ptr_->points.size() << "\n";

    // Initialise the KdTreeFLANN for the old pointcloud
    pcl::KdTreeFLANN<pcl::PointXYZ> old_kd_tree_flann;
    old_kd_tree_flann.setInputCloud(old_pointcloud_ptr_);
    for (size_t i = 0; i < new_pointcloud_ptr_->points.size(); ++i)
    {
        pcl::PointXYZ search_point = new_pointcloud_ptr_->points[i];
        std::vector<int> k_indices;
        std::vector<float> k_squared_distances;
        float radius = min_moving_distance_;
        if (!(old_kd_tree_flann.radiusSearch(search_point, radius, k_indices, k_squared_distances) > 0))
        {
            new_diff_ptr_->points.push_back(search_point);
        }
    }
    new_diff_ptr_->width = new_diff_ptr_->points.size();
    new_diff_ptr_->height = 1;
    new_diff_ptr_->is_dense = false;
    std::cout << "New diff size: " << new_diff_ptr_->points.size() << "\n";
}

void CloudDiffChecker::get_pointcloud_parameters()
{
    old_resolution_ = CloudDiffChecker::compute_pointcloud_resolution(old_pointcloud_ptr_);
    new_resolution_ = CloudDiffChecker::compute_pointcloud_resolution(new_pointcloud_ptr_);
    std::cout << "old_resolution_ = " << old_resolution_ << "\n";
    std::cout << "new_resolution_ = " << new_resolution_ << "\n";

    // Determine the pointclouds' boundaries
    for (size_t i = 0; i < old_pointcloud_ptr_->points.size(); ++i)
    {
        if (old_pointcloud_ptr_->points[i].x < x_min_)
        {
            x_min_ = old_pointcloud_ptr_->points[i].x;
        }
        if (old_pointcloud_ptr_->points[i].y < y_min_)
        {
            y_min_ = old_pointcloud_ptr_->points[i].y;
        }
        if (old_pointcloud_ptr_->points[i].z < z_min_)
        {
            z_min_ = old_pointcloud_ptr_->points[i].z;
        }
        if (old_pointcloud_ptr_->points[i].x > x_max_)
        {
            x_max_ = old_pointcloud_ptr_->points[i].x;
        }
        if (old_pointcloud_ptr_->points[i].y > y_max_)
        {
            y_max_ = old_pointcloud_ptr_->points[i].y;
        }
        if (old_pointcloud_ptr_->points[i].z > z_max_)
        {
            z_max_ = old_pointcloud_ptr_->points[i].z;
        }
    }
    for (size_t i = 0; i < new_pointcloud_ptr_->points.size(); ++i)
    {
        if (new_pointcloud_ptr_->points[i].x < x_min_)
        {
            x_min_ = new_pointcloud_ptr_->points[i].x;
        }
        if (new_pointcloud_ptr_->points[i].y < y_min_)
        {
            y_min_ = new_pointcloud_ptr_->points[i].y;
        }
        if (new_pointcloud_ptr_->points[i].z < z_min_)
        {
            z_min_ = new_pointcloud_ptr_->points[i].z;
        }
        if (new_pointcloud_ptr_->points[i].x > x_max_)
        {
            x_max_ = new_pointcloud_ptr_->points[i].x;
        }
        if (new_pointcloud_ptr_->points[i].y > y_max_)
        {
            y_max_ = new_pointcloud_ptr_->points[i].y;
        }
        if (new_pointcloud_ptr_->points[i].z > z_max_)
        {
            z_max_ = new_pointcloud_ptr_->points[i].z;
        }
    }

    std::cout << "x_min_ = " << x_min_ << " | x_max_ = " << x_max_ << " | y_min_ = " << y_min_ << " | y_max_ = "
              << y_max_ << " | z_min_ = " << z_min_ << " | z_max_ = " << z_max_ << "\n";

    // Get the longest direction of the pointcloud
    d_max_ = x_max_ - x_min_;
    if (y_max_ - y_min_ > d_max_)
    {
        d_max_ = y_max_ - y_min_;
    }
    if (z_max_ - z_min_ > d_max_)
    {
        d_max_ = z_max_ - z_min_;
    }
    std::cout << "d_max_ = " << d_max_ << "\n";
    min_moving_distance_ = d_max_ / 500;
    std::cout << "min_moving_distance_ = " << min_moving_distance_ << "\n";

    float d_x = x_max_ - x_min_;
    float d_y = y_max_ - y_min_;
    if (d_x < d_y)
    {
        grid_step_length_ = d_x / grid_count_;
    }
    else
    {
        grid_step_length_ = d_y / grid_count_;
    }
    std::cout << "grid_step_length_ = " << grid_step_length_ << "\n";

    // Determine the pointcloud raster size
    x_grid_count_ = ceil(d_x / grid_step_length_);
    y_grid_count_ = ceil(d_y / grid_step_length_);
    std::cout << "x_grid_count_ = " << x_grid_count_ << "\n";
    std::cout << "y_grid_count_ = " << y_grid_count_ << "\n";

    ransac_distance_threshold_ = grid_step_length_ / 10;
    std::cout << "ransac_distance_threshold_: " << ransac_distance_threshold_ << "\n";
}

void CloudDiffChecker::gridding_diff()
{
    std::cout << "gridding_diff.\n";

    // Grid the different regions on the new pointcloud
    std::vector<std::vector<pcl::PointXYZ>> new_diff_grid_ptr(x_grid_count_ * y_grid_count_, std::vector<pcl::PointXYZ>());
    for (size_t i = 0; i < new_diff_ptr_->points.size(); ++i)
    {
        int x_grid_index = std::min((int)std::floor((new_diff_ptr_->points[i].x - x_min_) / grid_step_length_), x_grid_count_ - 1);
        int y_grid_index = std::min((int)std::floor((new_diff_ptr_->points[i].y - y_min_) / grid_step_length_), y_grid_count_ - 1);
        new_diff_grid_ptr[y_grid_index * x_grid_count_ + x_grid_index].push_back(
            pcl::PointXYZ(new_diff_ptr_->points[i].x, new_diff_ptr_->points[i].y, new_diff_ptr_->points[i].z));
    }

    // Grid the different regions on the old pointcloud
    std::vector<std::vector<pcl::PointXYZ>> old_diff_grid_ptr(x_grid_count_ * y_grid_count_, std::vector<pcl::PointXYZ>());
    for (size_t i = 0; i < old_diff_ptr_->points.size(); ++i)
    {
        int x_grid_index = std::min((int)std::floor((old_diff_ptr_->points[i].x - x_min_) / grid_step_length_), x_grid_count_ - 1);
        int y_grid_index = std::min((int)std::floor((old_diff_ptr_->points[i].y - y_min_) / grid_step_length_), y_grid_count_ - 1);
        old_diff_grid_ptr[y_grid_index * x_grid_count_ + x_grid_index].push_back(
            pcl::PointXYZ(old_diff_ptr_->points[i].x, old_diff_ptr_->points[i].y, old_diff_ptr_->points[i].z));
    }

    cv::Mat grid_image = cv::Mat::zeros(y_grid_count_, x_grid_count_, CV_8UC1);
    for (size_t i = 0; i < new_diff_grid_ptr.size(); ++i)
    {
        int x_index = i % x_grid_count_;
        int y_index = i / x_grid_count_;
        if (new_diff_grid_ptr[i].size() > min_points_in_grid_ &&
            old_diff_grid_ptr[i].size() > min_points_in_grid_)
        {
            grid_image.at<uchar>(y_index, x_index) = 255;
        }
    }
    std::cout << "gridding_diff done!\n";

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(grid_image.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    std::cout << "contours.size() = " << contours.size() << "\n";
    cv::Mat contours_image = cv::Mat(y_grid_count_, x_grid_count_, CV_8UC1, 255);
    for (size_t i = 0; i < contours.size(); ++i)
    {
        cv::drawContours(contours_image, contours, i, i, CV_FILLED);
    }

    // Get the cluster's indices
    cluster_indices_ = std::vector<int>(x_grid_count_ * y_grid_count_, -1);
    for (size_t i = 0; i < new_diff_grid_ptr.size(); ++i)
    {
        int x_index = i % x_grid_count_;
        int y_index = i / x_grid_count_;
        if ((int)contours_image.at<uchar>(y_index, x_index) == 255)
        {
            cluster_indices_[i] = -1;
        }
        else
        {
            cluster_indices_[i] = (int)contours_image.at<uchar>(y_index, x_index);
        }
    }

    // Index the points to fit the planes
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> old_cloud(contours.size());
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> new_cloud(contours.size());
    for (size_t i = 0; i < contours.size(); ++i)
    {
        old_cloud[i] = (pcl::PointCloud<pcl::PointXYZ>::Ptr)(new pcl::PointCloud<pcl::PointXYZ>());
        new_cloud[i] = (pcl::PointCloud<pcl::PointXYZ>::Ptr)(new pcl::PointCloud<pcl::PointXYZ>());
    }

    for (size_t i = 0; i < new_diff_grid_ptr.size(); ++i)
    {
        if (cluster_indices_[i] >= 0)
        {
            for (size_t j = 0; j < old_diff_grid_ptr[i].size(); ++j)
            {
                old_cloud[cluster_indices_[i]]->points.push_back(old_diff_grid_ptr[i][j]);
            }
            for (size_t j = 0; j < new_diff_grid_ptr[i].size(); ++j)
            {
                new_cloud[cluster_indices_[i]]->points.push_back(new_diff_grid_ptr[i][j]);
            }
        }
    }
    for (size_t i = 0; i < contours.size(); ++i)
    {
        old_cloud[i]->width = old_cloud[i]->points.size();
        old_cloud[i]->height = 1;
        old_cloud[i]->is_dense = false;
        new_cloud[i]->width = new_cloud[i]->points.size();
        new_cloud[i]->height = 1;
        new_cloud[i]->is_dense = false;
    }

    // Frame the pointclouds to vectors
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> resized_old_cloud(contours.size());
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> resized_new_cloud(contours.size());
    for (size_t i = 0; i < contours.size(); ++i)
    {
        resized_old_cloud[i] = (pcl::PointCloud<pcl::PointXYZ>::Ptr)(new pcl::PointCloud<pcl::PointXYZ>());
        resized_new_cloud[i] = (pcl::PointCloud<pcl::PointXYZ>::Ptr)(new pcl::PointCloud<pcl::PointXYZ>());
    }
    for (size_t i = 0; i < new_diff_grid_ptr.size(); ++i)
    {
        if (cluster_indices_[i] >= 0)
        {
            for (size_t j = 0; j < old_diff_grid_ptr[i].size(); ++j)
            {
                resized_old_cloud[cluster_indices_[i]]->points.push_back(old_diff_grid_ptr[i][j]);
            }
            for (size_t j = 0; j < new_diff_grid_ptr[i].size(); ++j)
            {
                resized_new_cloud[cluster_indices_[i]]->points.push_back(new_diff_grid_ptr[i][j]);
            }
        }
    }

    // Take randdomly 5000 points from the pointclouds for plane fitting
    for (size_t i = 0; i < contours.size(); ++i)
    {
        if (resized_old_cloud[i]->points.size() > 5000)
        {
            std::random_shuffle(resized_old_cloud[i]->points.begin(), resized_old_cloud[i]->points.end());
            resized_old_cloud[i]->points.resize(5000);
        }
        resized_old_cloud[i]->width = resized_old_cloud[i]->points.size();
        resized_old_cloud[i]->height = 1;
        resized_old_cloud[i]->is_dense = false;
        if (resized_new_cloud[i]->points.size() > 5000)
        {
            std::random_shuffle(resized_new_cloud[i]->points.begin(), resized_new_cloud[i]->points.end());
            resized_new_cloud[i]->points.resize(5000);
        }
        resized_new_cloud[i]->width = resized_new_cloud[i]->points.size();
        resized_new_cloud[i]->height = 1;
        resized_new_cloud[i]->is_dense = false;
    }

    // Fit planes to the old pointcloud
    plane_coefficients_ = std::vector<PlaneCoefficients>(contours.size(), PlaneCoefficients());
    for (size_t i = 0; i < contours.size(); ++i)
    {
        std::vector<int> inliers;
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr old_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(resized_old_cloud[i]));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(old_plane);
        ransac.setDistanceThreshold(ransac_distance_threshold_);
        ransac.computeModel();
        Eigen::VectorXf coefficients;
        ransac.getModelCoefficients(coefficients);
        ransac.getInliers(inliers);
        if (coefficients[2] < 0)
        {
            for (int j = 0; j < 4; ++j)
            {
                coefficients[j] = -coefficients[j];
            }
        }
        plane_coefficients_[i].old_a = coefficients[0];
        plane_coefficients_[i].old_b = coefficients[1];
        plane_coefficients_[i].old_c = coefficients[2];
        plane_coefficients_[i].old_d = coefficients[3];
        plane_coefficients_[i].old_inliers = inliers.size();
        plane_coefficients_[i].old_total = resized_old_cloud[i]->points.size();
    }
    std::cout << "Fit old planes, done!\n";

    // Fit planes to the new pointcloud
    for (size_t i = 0; i < contours.size(); ++i)
    {
        std::vector<int> inliers;
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr new_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(resized_new_cloud[i]));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(new_plane);
        ransac.setDistanceThreshold(ransac_distance_threshold_);
        ransac.computeModel();
        Eigen::VectorXf coefficients;
        ransac.getModelCoefficients(coefficients);
        ransac.getInliers(inliers);
        if (coefficients[2] < 0)
        {
            for (int j = 0; j < 4; ++j)
            {
                coefficients[j] = -coefficients[j];
            }
        }
        plane_coefficients_[i].new_a = coefficients[0];
        plane_coefficients_[i].new_b = coefficients[1];
        plane_coefficients_[i].new_c = coefficients[2];
        plane_coefficients_[i].new_d = coefficients[3];
        plane_coefficients_[i].new_inliers = inliers.size();
        plane_coefficients_[i].new_total = resized_new_cloud[i]->points.size();
    }
    std::cout << "Fit new planes, done!\n";
}

void CloudDiffChecker::get_refer_plane()
{
    std::vector<ReferPlane> duplicated_refer_planes;
    duplicated_refer_planes = std::vector<ReferPlane>(x_grid_count_ * y_grid_count_, ReferPlane());
    for (size_t i = 0; i < duplicated_refer_planes.size(); ++i)
    {
        int x_index = i % x_grid_count_;
        int y_index = i / x_grid_count_;
        float centroid_x = x_min_ + x_index * grid_step_length_ + grid_step_length_ / 2;
        float centroid_y = y_min_ + y_index * grid_step_length_ + grid_step_length_ / 2;
        float new_centroid_z = -(plane_coefficients_[cluster_indices_[i]].new_a * centroid_x + plane_coefficients_[cluster_indices_[i]].new_b * centroid_y + plane_coefficients_[cluster_indices_[i]].new_d) / plane_coefficients_[cluster_indices_[i]].new_c;
        float old_centroid_z = -(plane_coefficients_[cluster_indices_[i]].old_a * centroid_x + plane_coefficients_[cluster_indices_[i]].old_b * centroid_y + plane_coefficients_[cluster_indices_[i]].old_d) / plane_coefficients_[cluster_indices_[i]].old_c;
        float normal_refer_x = plane_coefficients_[cluster_indices_[i]].new_a + plane_coefficients_[cluster_indices_[i]].old_a;
        float normal_refer_y = plane_coefficients_[cluster_indices_[i]].new_b + plane_coefficients_[cluster_indices_[i]].old_b;
        float normal_refer_z = plane_coefficients_[cluster_indices_[i]].new_c + plane_coefficients_[cluster_indices_[i]].old_c;
        float normal_refer_length = sqrt(normal_refer_x * normal_refer_x + normal_refer_y * normal_refer_y + normal_refer_z * normal_refer_z);
        normal_refer_x /= normal_refer_length;
        normal_refer_y /= normal_refer_length;
        normal_refer_z /= normal_refer_length;
        float centroid_z = (new_centroid_z + old_centroid_z) / 2;
        duplicated_refer_planes[i].refer_a = normal_refer_x;
        duplicated_refer_planes[i].refer_b = normal_refer_y;
        duplicated_refer_planes[i].refer_c = normal_refer_z;
        duplicated_refer_planes[i].refer_d = -(normal_refer_x * centroid_x + normal_refer_y * centroid_y + normal_refer_z * centroid_z);
    }

    // Get the refer planes
    refer_planes_ = std::vector<ReferPlane>(plane_coefficients_.size(), ReferPlane());
    std::vector<bool> is_setup(plane_coefficients_.size(), false);
    for (size_t i = 0; i < duplicated_refer_planes.size(); ++i)
    {
        int cluster_index = cluster_indices_[i];
        if (cluster_index >= 0)
        {
            if (!is_setup[cluster_index])
            {
                refer_planes_[cluster_index].refer_a = duplicated_refer_planes[i].refer_a;
                refer_planes_[cluster_index].refer_b = duplicated_refer_planes[i].refer_b;
                refer_planes_[cluster_index].refer_c = duplicated_refer_planes[i].refer_c;
                refer_planes_[cluster_index].refer_d = duplicated_refer_planes[i].refer_d;
                is_setup[cluster_index] = true;
            }
        }
    }
    std::cout << "Get refer planes, done!\n";
}

void CloudDiffChecker::get_projections()
{
    old_projection_ptr_ = (pcl::PointCloud<pcl::PointXYZ>::Ptr)(new pcl::PointCloud<pcl::PointXYZ>());
    new_projection_ptr_ = (pcl::PointCloud<pcl::PointXYZ>::Ptr)(new pcl::PointCloud<pcl::PointXYZ>());

    // Get the projection on the old pointcloud
    for (size_t i = 0; i < old_pointcloud_ptr_->points.size(); ++i)
    {
        int x_grid_index = std::min((int)std::floor((old_pointcloud_ptr_->points[i].x - x_min_) / grid_step_length_), x_grid_count_ - 1);
        int y_grid_index = std::min((int)std::floor((old_pointcloud_ptr_->points[i].y - y_min_) / grid_step_length_), y_grid_count_ - 1);
        int grid_index = y_grid_index * x_grid_count_ + x_grid_index;
        int cluster_index = cluster_indices_[grid_index];
        pcl::PointXYZ projection;
        if (cluster_index >= 0)
        {
            pcl::PointXYZ point_xyz = old_pointcloud_ptr_->points[i];
            pcl::PointXYZ normal;
            normal.x = refer_planes_[cluster_index].refer_a;
            normal.y = refer_planes_[cluster_index].refer_b;
            normal.z = refer_planes_[cluster_index].refer_c;
            projection = line_onto_plane(point_xyz, normal, refer_planes_[cluster_index].refer_a, refer_planes_[cluster_index].refer_b, refer_planes_[cluster_index].refer_c, refer_planes_[cluster_index].refer_d);
        }
        else
        {
            // Outliers
            projection.x = 2 * x_min_ - x_max_;
            projection.y = 2 * y_min_ - y_max_;
            projection.z = 2 * z_min_ - z_max_;
        }
        old_projection_ptr_->points.push_back(projection);
    }
    old_projection_ptr_->width = old_projection_ptr_->points.size();
    old_projection_ptr_->height = 1;
    old_projection_ptr_->is_dense = false;

    // Get the projection on the new pointcloud
    for (size_t i = 0; i < new_pointcloud_ptr_->points.size(); ++i)
    {
        int x_grid_index = std::min((int)std::floor((new_pointcloud_ptr_->points[i].x - x_min_) / grid_step_length_), x_grid_count_ - 1);
        int y_grid_index = std::min((int)std::floor((new_pointcloud_ptr_->points[i].y - y_min_) / grid_step_length_), y_grid_count_ - 1);
        int grid_index = y_grid_index * x_grid_count_ + x_grid_index;
        int cluster_index = cluster_indices_[grid_index];
        pcl::PointXYZ projection;
        if (cluster_index >= 0)
        {
            pcl::PointXYZ point_xyz = new_pointcloud_ptr_->points[i];
            pcl::PointXYZ normal;
            normal.x = refer_planes_[cluster_index].refer_a;
            normal.y = refer_planes_[cluster_index].refer_b;
            normal.z = refer_planes_[cluster_index].refer_c;
            projection = line_onto_plane(point_xyz, normal, refer_planes_[cluster_index].refer_a, refer_planes_[cluster_index].refer_b, refer_planes_[cluster_index].refer_c, refer_planes_[cluster_index].refer_d);
        }
        else
        {
            // Outliers
            projection.x = 2 * x_min_ - x_max_;
            projection.y = 2 * y_min_ - y_max_;
            projection.z = 2 * z_min_ - z_max_;
        }
        new_projection_ptr_->points.push_back(projection);
    }
    new_projection_ptr_->width = new_projection_ptr_->points.size();
    new_projection_ptr_->height = 1;
    new_projection_ptr_->is_dense = false;
    std::cout << "Get projections, done!\n";
}

void CloudDiffChecker::draw_transformation()
{
    std::vector<pcl::PointXYZRGB> ply_scene_cloud;
    pcl::PointXYZRGB point_xyzrgb;
    point_xyzrgb.r = 0;
    point_xyzrgb.g = 0;
    point_xyzrgb.b = 255;
    for (size_t i = 0; i < old_pointcloud_ptr_->points.size(); ++i)
    {
        point_xyzrgb.x = old_pointcloud_ptr_->points[i].x;
        point_xyzrgb.y = old_pointcloud_ptr_->points[i].y;
        point_xyzrgb.z = old_pointcloud_ptr_->points[i].z;
        ply_scene_cloud.push_back(point_xyzrgb);
    }
    point_xyzrgb.r = 255;
    point_xyzrgb.g = 0;
    point_xyzrgb.b = 0;
    for (size_t i = 0; i < new_pointcloud_ptr_->points.size(); ++i)
    {
        point_xyzrgb.x = new_pointcloud_ptr_->points[i].x;
        point_xyzrgb.y = new_pointcloud_ptr_->points[i].y;
        point_xyzrgb.z = new_pointcloud_ptr_->points[i].z;
        ply_scene_cloud.push_back(point_xyzrgb);
    }

    // Initialise the KdTreeFLANN for the old projection pointcloud
    pcl::KdTreeFLANN<pcl::PointXYZ> old_kd_tree_flann_projection;
    old_kd_tree_flann_projection.setInputCloud(old_projection_ptr_);

    // Initialise the KdTreeFLANN for the new projection pointcloud
    pcl::KdTreeFLANN<pcl::PointXYZ> new_kd_tree_flann_projection;
    new_kd_tree_flann_projection.setInputCloud(new_projection_ptr_);

    for (size_t i = 0; i < cluster_indices_.size(); ++i)
    {
        if (cluster_indices_[i] >= 0)
        {
            int x_index = i % x_grid_count_;
            int y_index = i / x_grid_count_;

            // Draw the transform vectors
            point_xyzrgb.r = 255;
            point_xyzrgb.g = 255;
            point_xyzrgb.b = 255;
            for (float x = x_min_ + x_index * grid_step_length_; x < x_min_ + x_index * grid_step_length_ + grid_step_length_; x += grid_step_length_ / 2)
            {
                for (float y = y_min_ + y_index * grid_step_length_; y < y_min_ + y_index * grid_step_length_ + grid_step_length_; y += grid_step_length_ / 2)
                {
                    float z = -(refer_planes_[cluster_indices_[i]].refer_a * x + refer_planes_[cluster_indices_[i]].refer_b * y +
                                refer_planes_[cluster_indices_[i]].refer_d) /
                              refer_planes_[cluster_indices_[i]].refer_c;
                    pcl::PointXYZ search_point;
                    search_point.x = x;
                    search_point.y = y;
                    search_point.z = z;
                    std::vector<int> k_indices;
                    std::vector<float> k_squared_distances;
                    size_t min_size_projection = 0;
                    float radius = grid_step_length_ / 2;
                    pcl::PointXYZ old_terminal(0, 0, 0);
                    float old_stdv = 0;
                    if ((old_kd_tree_flann_projection.radiusSearch(search_point, radius, k_indices, k_squared_distances) > 0))
                    {
                        for (size_t j = 0; j < k_indices.size(); ++j)
                        {
                            old_terminal.x += old_pointcloud_ptr_->points[k_indices[j]].x;
                            old_terminal.y += old_pointcloud_ptr_->points[k_indices[j]].y;
                            old_terminal.z += old_pointcloud_ptr_->points[k_indices[j]].z;
                        }
                        old_terminal.x /= k_indices.size();
                        old_terminal.y /= k_indices.size();
                        old_terminal.z /= k_indices.size();
                        for (size_t j = 0; j < k_indices.size(); ++j)
                        {
                            pcl::PointXYZ old_point = old_pointcloud_ptr_->points[k_indices[j]];
                            old_stdv += squared_distance(old_terminal, old_point);
                        }
                        min_size_projection = k_indices.size();
                        old_stdv /= k_indices.size();
                        old_stdv = sqrt(old_stdv);
                    }
                    k_indices.clear();
                    k_squared_distances.clear();
                    pcl::PointXYZ new_terminal(0, 0, 0);
                    float new_stdv = 0;
                    if ((new_kd_tree_flann_projection.radiusSearch(search_point, radius, k_indices, k_squared_distances) > 0))
                    {
                        for (size_t j = 0; j < k_indices.size(); ++j)
                        {
                            new_terminal.x += new_pointcloud_ptr_->points[k_indices[j]].x;
                            new_terminal.y += new_pointcloud_ptr_->points[k_indices[j]].y;
                            new_terminal.z += new_pointcloud_ptr_->points[k_indices[j]].z;
                        }
                        new_terminal.x /= k_indices.size();
                        new_terminal.y /= k_indices.size();
                        new_terminal.z /= k_indices.size();
                        for (size_t j = 0; j < k_indices.size(); ++j)
                        {
                            pcl::PointXYZ new_point = new_pointcloud_ptr_->points[k_indices[j]];
                            new_stdv += squared_distance(new_terminal, new_point);
                        }
                        new_stdv /= k_indices.size();
                        new_stdv = sqrt(new_stdv);
                        if (k_indices.size() < min_size_projection)
                        {
                            min_size_projection = k_indices.size();
                        }
                    }

                    if (old_terminal.x != 0 && new_terminal.x != 0 && old_stdv < d_max_ / 250 && new_stdv < d_max_ / 250 && min_size_projection > 1)
                    {
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
                        for (float t = 0; t < FLT_MAX; t += grid_step_length_ / 100)
                        {
                            point_xyzrgb.x = old_terminal.x + t * vec.x;
                            point_xyzrgb.y = old_terminal.y + t * vec.y;
                            point_xyzrgb.z = old_terminal.z + t * vec.z;
                            ply_scene_cloud.push_back(point_xyzrgb);
                            if (t > length)
                            {
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformation(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (size_t i = 0; i < ply_scene_cloud.size(); ++i)
    {
        transformation->points.push_back(ply_scene_cloud[i]);
    }
    transformation->width = ply_scene_cloud.size();
    transformation->height = 1;
    pcl::io::savePLYFile(matching_results_file_, *transformation, true);
    std::cout << matching_results_file_ << " saved.\n";
}
