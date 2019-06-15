#include "pointcloud_matching/cloud_projection.hpp"

void normalise_colours(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_ptr) {
    // Compute the pointcloud's average corlours
    double r_avg = 0, g_avg = 0, b_avg = 0;
    for (size_t i = 0; i < pcl_ptr->points.size(); ++i) {
        r_avg += pcl_ptr->points[i].r;
        g_avg += pcl_ptr->points[i].g;
        b_avg += pcl_ptr->points[i].b;
    }
    r_avg /= pcl_ptr->points.size();
    g_avg /= pcl_ptr->points.size();
    b_avg /= pcl_ptr->points.size();

    // Compute colours' standard deviations
    double r_std = 0, g_std = 0, b_std = 0;
    for (size_t i = 0; i < pcl_ptr->points.size(); ++i) {
        r_std += (r_avg - pcl_ptr->points[i].r) * (r_avg - pcl_ptr->points[i].r);
        g_std += (g_avg - pcl_ptr->points[i].g) * (g_avg - pcl_ptr->points[i].g);
        b_std += (b_avg - pcl_ptr->points[i].b) * (b_avg - pcl_ptr->points[i].b);
    }
    r_std /= pcl_ptr->points.size();
    g_std /= pcl_ptr->points.size();
    b_std /= pcl_ptr->points.size();
    r_std = sqrt(r_std);
    g_std = sqrt(g_std);
    b_std = sqrt(b_std);

    // Synchronise pointclouds' colours
    for (size_t i = 0; i < pcl_ptr->points.size(); ++i) {
        pcl_ptr->points[i].r = std::max(0.0, std::min(255.0, (100.0 + (pcl_ptr->points[i].r - r_avg) * 35.0 / r_std)));
        pcl_ptr->points[i].g = std::max(0.0, std::min(255.0, (100.0 + (pcl_ptr->points[i].g - g_avg) * 35.0 / g_std)));
        pcl_ptr->points[i].b = std::max(0.0, std::min(255.0, (100.0 + (pcl_ptr->points[i].b - b_avg) * 35.0 / b_std)));
    }
}

CloudProjection::CloudProjection(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr old_pcl_ptr,
                                 const pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_pcl_ptr,
                                 const pcl::PointCloud<pcl::PointXYZRGB>::Ptr old_parts_ptr,
                                 const pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_parts_ptr) {
    old_pcl_ptr_ = old_pcl_ptr;
    new_pcl_ptr_ = new_pcl_ptr;
    old_parts_ptr_ = old_parts_ptr;
    new_parts_ptr_ = new_parts_ptr;
    match_train_indices_.clear();
    match_query_indices_.clear();
    direction_indices_.clear();
}

CloudProjection::~CloudProjection() {
}

void CloudProjection::get_matches_by_direction(const Eigen::Matrix4f transform, const int direction_index) {
    // Rotate the old pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotated_old_pcl_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*old_pcl_ptr_, *rotated_old_pcl_ptr, transform);

    // Rotate the new pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotated_new_pcl_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*new_pcl_ptr_, *rotated_new_pcl_ptr, transform);

    // Initialise the KdTree search for normal estimation
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(20);

    // Estimate the normals of the rotated old pointcloud
    pcl::PointCloud<pcl::Normal>::Ptr rotated_old_normal_ptr(new pcl::PointCloud<pcl::Normal>());
    norm_est.setInputCloud(rotated_old_pcl_ptr);
    norm_est.compute(*rotated_old_normal_ptr);
    std::cout << "Rotated old pointcloud normal size:" << rotated_old_normal_ptr->points.size() << "\n";

    // Estimate the normals of the rotated new pointcloud
    pcl::PointCloud<pcl::Normal>::Ptr rotated_new_normal_ptr(new pcl::PointCloud<pcl::Normal>());
    norm_est.setInputCloud(rotated_new_pcl_ptr);
    norm_est.compute(*rotated_new_normal_ptr);
    std::cout << "Rotated new pointcloud normal size:" << rotated_new_normal_ptr->points.size() << "\n";

    // Flaten the rotated old pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr flattened_old_pcl_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (size_t i = 0; i < rotated_old_pcl_ptr->points.size(); ++i) {
        pcl::PointXYZRGB tmp = rotated_old_pcl_ptr->points[i];
        tmp.z = 0;
        flattened_old_pcl_ptr->points.push_back(tmp);
    }
    flattened_old_pcl_ptr->width = flattened_old_pcl_ptr->points.size();
    flattened_old_pcl_ptr->height = 1;
    flattened_old_pcl_ptr->is_dense = 1;
    std::cout << "flattened_old_pcl_ptr size: " << flattened_old_pcl_ptr->points.size() << "\n";

    // Flaten the rotated new pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr flattened_new_pcl_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (size_t i = 0; i < rotated_new_pcl_ptr->points.size(); ++i) {
        pcl::PointXYZRGB tmp = rotated_new_pcl_ptr->points[i];
        tmp.z = 0;
        flattened_new_pcl_ptr->points.push_back(tmp);
    }
    flattened_new_pcl_ptr->width = flattened_new_pcl_ptr->points.size();
    flattened_new_pcl_ptr->height = 1;
    flattened_new_pcl_ptr->is_dense = 1;
    std::cout << "flattened_new_pcl_ptr size: " << flattened_new_pcl_ptr->points.size() << "\n";

    // Determine the pointcloud's boundary
    double x_min = 1e10, y_min = 1e10, z_min = 1e10;
    double x_max = -1e10, y_max = -1e10, z_max = -1e10;
    for (size_t i = 0; i < rotated_old_pcl_ptr->points.size(); ++i) {
        if (rotated_old_pcl_ptr->points[i].x < x_min)
            x_min = rotated_old_pcl_ptr->points[i].x;
        if (rotated_old_pcl_ptr->points[i].y < y_min)
            y_min = rotated_old_pcl_ptr->points[i].y;
        if (rotated_old_pcl_ptr->points[i].z < z_min)
            z_min = rotated_old_pcl_ptr->points[i].z;
        if (rotated_old_pcl_ptr->points[i].x > x_max)
            x_max = rotated_old_pcl_ptr->points[i].x;
        if (rotated_old_pcl_ptr->points[i].y > y_max)
            y_max = rotated_old_pcl_ptr->points[i].y;
        if (rotated_old_pcl_ptr->points[i].z > z_max)
            z_max = rotated_old_pcl_ptr->points[i].z;
    }
    for (size_t i = 0; i < rotated_new_pcl_ptr->points.size(); ++i) {
        if (rotated_new_pcl_ptr->points[i].x < x_min)
            x_min = rotated_new_pcl_ptr->points[i].x;
        if (rotated_new_pcl_ptr->points[i].y < y_min)
            y_min = rotated_new_pcl_ptr->points[i].y;
        if (rotated_new_pcl_ptr->points[i].z < z_min)
            z_min = rotated_new_pcl_ptr->points[i].z;
        if (rotated_new_pcl_ptr->points[i].x > x_max)
            x_max = rotated_new_pcl_ptr->points[i].x;
        if (rotated_new_pcl_ptr->points[i].y > y_max)
            y_max = rotated_new_pcl_ptr->points[i].y;
        if (rotated_new_pcl_ptr->points[i].z > z_max)
            z_max = rotated_new_pcl_ptr->points[i].z;
    }
    std::cout << "x: " << x_min << " -> " << x_max << "\n";
    std::cout << "y: " << y_min << " -> " << y_max << "\n";
    std::cout << "z: " << z_min << " -> " << z_max << "\n";
    double dx = x_max - x_min;
    double dy = y_max - y_min;
    double d_min = std::min(dx, dy);
    std::cout << "d_min: " << d_min << "\n";

    // Initialise the KdTreeFLANN search for the flattened old pointcloud radiusSearch
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_flattened_old_pcl;
    kd_flattened_old_pcl.setInputCloud(flattened_old_pcl_ptr);

    // Find the image's resolution
    int low_size = 0;
    int high_size = 1500;
    for (int l = 0; l < 10; ++l) {
        int optimal_size = (low_size + high_size) / 2;
        double optimal_threshold = d_min / optimal_size;
        int optimal_x_size = floor(dx / optimal_threshold) + 1;
        int optimal_y_size = floor(dy / optimal_threshold) + 1;
        cv::Mat count_mat = cv::Mat::zeros(optimal_y_size, optimal_x_size, CV_32SC1);
        for (int i = 0; i < count_mat.rows; ++i) {
            for (int j = 0; j < count_mat.cols; ++j) {
                pcl::PointXYZRGB tmp;
                tmp.x = x_min + j * optimal_threshold + optimal_threshold / 2;
                tmp.y = y_min + i * optimal_threshold + optimal_threshold / 2;
                tmp.z = 0;
                std::vector<int> nn_index;
                std::vector<float> nn_sqd_distance;
                if (!(kd_flattened_old_pcl.radiusSearch(tmp, 0.5 * optimal_threshold, nn_index, nn_sqd_distance) > 0)) {
                    continue;
                }
                if (nn_index.size()) {
                    ++count_mat.at<int>(i, j);
                }
            }
        }
        int single_count = cv::countNonZero(count_mat);

        count_mat = cv::Mat::zeros(optimal_y_size, optimal_x_size, CV_32SC1);
        for (int i = 0; i < count_mat.rows; ++i) {
            for (int j = 0; j < count_mat.cols; ++j) {
                pcl::PointXYZRGB tmp;
                tmp.x = x_min + j * optimal_threshold + optimal_threshold / 2;
                tmp.y = y_min + i * optimal_threshold + optimal_threshold / 2;
                tmp.z = 0;
                std::vector<int> nn_index;
                std::vector<float> nn_sqd_distance;
                if (!(kd_flattened_old_pcl.radiusSearch(tmp, M_SQRT1_2 * optimal_threshold, nn_index, nn_sqd_distance) > 0)) {
                    continue;
                }
                if (nn_index.size()) {
                    ++count_mat.at<int>(i, j);
                }
            }
        }
        int double_count = cv::countNonZero(count_mat);
        float dense_ratio = (float)single_count / double_count;
        std::cout << "dense_ratio: " << dense_ratio << std::endl;
        if (dense_ratio < 0.85) {
            high_size = optimal_size;
        } else {
            low_size = optimal_size;
        }
    }
    std::cout << "low high " << low_size << " " << high_size << "\n";
    if (low_size < 200) {
        return;
    }
    double distance_threshold = d_min / low_size;
    int x_size = floor(dx / distance_threshold) + 1;
    int y_size = floor(dy / distance_threshold) + 1;
    std::cout << "image resolution: " << x_size << "x" << y_size << "\n";

    // Project the old pointcloud
    cv::Mat old_projection_img = cv::Mat::zeros(y_size, x_size, CV_8UC3);
    for (int i = 0; i < old_projection_img.rows; ++i) {
        for (int j = 0; j < old_projection_img.cols; ++j) {
            pcl::PointXYZRGB tmp;
            tmp.x = x_min + j * distance_threshold + distance_threshold / 2;
            tmp.y = y_min + i * distance_threshold + distance_threshold / 2;
            tmp.z = 0;
            std::vector<int> nn_index;
            std::vector<float> nn_sqd_distance;
            if (!(kd_flattened_old_pcl.radiusSearch(tmp, distance_threshold, nn_index, nn_sqd_distance) > 0)) {
                continue;
            }
            pcl::PointXYZRGB nn_point = rotated_old_pcl_ptr->points[nn_index[0]];
            double max_z = -1e10;
            for (size_t k = 0; k < nn_index.size(); ++k) {
                if (rotated_old_pcl_ptr->points[nn_index[k]].z > max_z) {
                    max_z = rotated_old_pcl_ptr->points[nn_index[k]].z;
                    nn_point = rotated_old_pcl_ptr->points[nn_index[k]];
                }
            }
            old_projection_img.at<cv::Vec3b>(i, j)[0] = nn_point.b;
            old_projection_img.at<cv::Vec3b>(i, j)[1] = nn_point.g;
            old_projection_img.at<cv::Vec3b>(i, j)[2] = nn_point.r;
        }
    }

    // Initialise the KdTreeFLANN search for the flattened old pointcloud radiusSearch
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_flattened_new_pcl;
    kd_flattened_new_pcl.setInputCloud(flattened_new_pcl_ptr);

    // Project the new pointcloud
    cv::Mat new_projection_img = cv::Mat::zeros(y_size, x_size, CV_8UC3);
    for (int i = 0; i < new_projection_img.rows; ++i) {
        for (int j = 0; j < new_projection_img.cols; ++j) {
            pcl::PointXYZRGB tmp;
            tmp.x = x_min + j * distance_threshold + distance_threshold / 2;
            tmp.y = y_min + i * distance_threshold + distance_threshold / 2;
            tmp.z = 0;
            std::vector<int> nn_index;
            std::vector<float> nn_sqd_distance;
            if (!(kd_flattened_new_pcl.radiusSearch(tmp, distance_threshold, nn_index, nn_sqd_distance) > 0)) {
                continue;
            }
            pcl::PointXYZRGB nn_point = rotated_new_pcl_ptr->points[nn_index[0]];
            double max_z = -1e10;
            for (size_t k = 0; k < nn_index.size(); ++k) {
                if (rotated_new_pcl_ptr->points[nn_index[k]].z > max_z) {
                    max_z = rotated_new_pcl_ptr->points[nn_index[k]].z;
                    nn_point = rotated_new_pcl_ptr->points[nn_index[k]];
                }
            }
            new_projection_img.at<cv::Vec3b>(i, j)[0] = nn_point.b;
            new_projection_img.at<cv::Vec3b>(i, j)[1] = nn_point.g;
            new_projection_img.at<cv::Vec3b>(i, j)[2] = nn_point.r;
        }
    }

    // Detect the 2d matches
    std::vector<cv::Point2f> train_points, query_points;
    detect_2d_matches(old_projection_img, new_projection_img, distance_threshold, train_points, query_points, direction_index);

    // Get the 3d pairs
    std::cout << "OF 2d: " << train_points[0] << " " << query_points[0] << "\n";
    for (size_t i = 0; i < train_points.size(); ++i) {
        cv::Point2f train_point = train_points[i];
        cv::Point2f query_point = query_points[i];
        pcl::PointXYZRGB tmp;
        tmp.x = x_min + train_point.x * distance_threshold + distance_threshold / 2;
        tmp.y = y_min + train_point.y * distance_threshold + distance_threshold / 2;
        tmp.z = 0;
        std::vector<int> nn_index;
        std::vector<float> nn_sqd_distance;
        if (!(kd_flattened_old_pcl.radiusSearch(tmp, distance_threshold, nn_index, nn_sqd_distance) > 0)) {
            continue;
        }
        double max_z = -1e10;
        size_t old_best_nn_index = 0;
        pcl::PointXYZRGB nn_old_point = rotated_old_pcl_ptr->points[nn_index[0]];
        for (size_t j = 0; j < nn_index.size(); ++j) {
            if (rotated_old_pcl_ptr->points[nn_index[j]].z > max_z) {
                max_z = rotated_old_pcl_ptr->points[nn_index[j]].z;
                old_best_nn_index = nn_index[j];
                nn_old_point = rotated_old_pcl_ptr->points[old_best_nn_index];
            }
        }
        tmp.x = x_min + query_point.x * distance_threshold + distance_threshold / 2;
        tmp.y = y_min + query_point.y * distance_threshold + distance_threshold / 2;
        tmp.z = 0;
        if (!(kd_flattened_new_pcl.radiusSearch(tmp, distance_threshold, nn_index, nn_sqd_distance) > 0)) {
            continue;
        }
        max_z = -1e10;
        size_t new_best_nn_index = 0;
        pcl::PointXYZRGB nn_new_point = rotated_new_pcl_ptr->points[nn_index[0]];
        for (size_t j = 0; j < nn_index.size(); ++j) {
            if (rotated_new_pcl_ptr->points[nn_index[j]].z > max_z) {
                max_z = rotated_new_pcl_ptr->points[nn_index[j]].z;
                new_best_nn_index = nn_index[j];
                nn_new_point = rotated_new_pcl_ptr->points[new_best_nn_index];
            }
        }

        double dx = nn_old_point.x - nn_new_point.x;
        double dy = nn_old_point.y - nn_new_point.y;
        double dz = nn_old_point.z - nn_new_point.z;
        float length = sqrt(dx * dx + dy * dy + dz * dz);
        if (length > Configurations::get_instance()->pos_radius) {
            continue;
        }
        match_train_indices_.push_back(old_best_nn_index);
        match_query_indices_.push_back(new_best_nn_index);
        pcl::Normal old_normal = rotated_old_normal_ptr->points[old_best_nn_index];
        float old_normal_xy = sqrt(old_normal.normal_x * old_normal.normal_x + old_normal.normal_y * old_normal.normal_y);
        float old_normal_z = fabs(old_normal.normal_z);
        pcl::Normal new_normal = rotated_new_normal_ptr->points[new_best_nn_index];
        float new_normal_xy = sqrt(new_normal.normal_x * new_normal.normal_x + new_normal.normal_y * new_normal.normal_y);
        float new_normal_z = fabs(new_normal.normal_z);
        if (old_normal_z / old_normal_xy > 0.5 &&
            new_normal_z / new_normal_xy > 0.5) {
            direction_indices_.push_back(direction_index);
        } else {
            direction_indices_.push_back(-1);
        }
    }
}

void CloudProjection::detect_2d_matches(const cv::Mat old_projection_img,
                                        const cv::Mat new_projection_img,
                                        const double distance_threshold,
                                        std::vector<cv::Point2f>& train_points,
                                        std::vector<cv::Point2f>& query_points,
                                        const int direction_index) {
    //Detect key points using SIFT
    cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SIFT::create();
    std::vector<cv::KeyPoint> keypoints_old, keypoints_new;
    f2d->detect(old_projection_img, keypoints_old);
    std::cout << "keypoints_old's size " << keypoints_old.size() << "\n";
    f2d->detect(new_projection_img, keypoints_new);
    std::cout << "keypoints_new's size " << keypoints_new.size() << "\n";

    //Compute descriptors using SIFT
    cv::Mat descriptors_old, descriptors_new;
    f2d->compute(old_projection_img, keypoints_old, descriptors_old);
    f2d->compute(new_projection_img, keypoints_new, descriptors_new);
    int win_size = Configurations::get_instance()->pos_radius / distance_threshold;

    // Match the descriptors
    cv::FlannBasedMatcher matcher;
    std::vector<std::vector<cv::DMatch>> matches;
    matcher.knnMatch(descriptors_new, descriptors_old, matches, 2);
    std::vector<cv::DMatch> good_matches;
    for (size_t i = 0; i < matches.size(); ++i) {
        if (matches[i][0].distance / matches[i][1].distance < 0.7) {
            cv::Point2f train_point = keypoints_old[matches[i][0].trainIdx].pt;
            cv::Point2f query_point = keypoints_new[matches[i][0].queryIdx].pt;
            float dx = train_point.x - query_point.x;
            float dy = train_point.y - query_point.y;
            float d = sqrt(dx * dx + dy * dy);
            if (d < win_size) {
                good_matches.push_back(matches[i][0]);
            }
        }
    }
    std::cout << "good_matches.size() " << good_matches.size() << "\n";

    // Draw good matches
    cv::Mat correct_matches_img;
    drawMatches(new_projection_img, keypoints_new, old_projection_img, keypoints_old,
                good_matches, correct_matches_img, cv::Scalar::all(-1), cv::Scalar::all(-1),
                std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    // Detect good features to track
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01);
    cv::Size sub_pix_win_size(1, 1);
    cv::Mat old_projection_gray_img, new_projection_gray_img;
    std::vector<cv::Point2f> corners;
    cv::cvtColor(old_projection_img, old_projection_gray_img, CV_BGR2GRAY);
    cv::cvtColor(new_projection_img, new_projection_gray_img, CV_BGR2GRAY);
    cv::goodFeaturesToTrack(old_projection_gray_img, corners, 50000, 0.01, win_size / 2, cv::Mat(), 3, false, 0.04);

    if (corners.size() > 0) {
        cornerSubPix(old_projection_gray_img, corners, sub_pix_win_size, cv::Size(-1, -1), termcrit);
    }
    std::cout << "corners.size() " << corners.size() << "\n";

    cv::Mat corner_img = old_projection_img.clone();
    for (size_t i = 0; i < corners.size(); ++i) {
        cv::circle(corner_img, corners[i], 3, cv::Scalar(255), -1, 8);
    }

    // Execute optical flow
    std::vector<cv::Point2f> new_corners, tmp_corners;
    std::vector<uchar> status, status_2;
    std::vector<float> errors, errors_2;
    cv::Size winSize(win_size, win_size);
    cv::calcOpticalFlowPyrLK(old_projection_gray_img, new_projection_gray_img, corners, new_corners,
                             status, errors, winSize, 0, termcrit, 0, 0.0001);
    if (new_corners.size() > 0) {
        cornerSubPix(new_projection_gray_img, new_corners, sub_pix_win_size, cv::Size(-1, -1), termcrit);
    }
    cv::calcOpticalFlowPyrLK(new_projection_gray_img, old_projection_gray_img, new_corners, tmp_corners,
                             status_2, errors_2, winSize, 0, termcrit, 0, 0.0001);
    if (tmp_corners.size() > 0) {
        cornerSubPix(old_projection_gray_img, tmp_corners, sub_pix_win_size, cv::Size(-1, -1), termcrit);
    }
    float OF_error_threshold = Configurations::get_instance()->OF_error_threshold;
    for (size_t i = 0; i < corners.size(); ++i) {
        float dx_err = corners[i].x - tmp_corners[i].x;
        float dy_err = corners[i].y - tmp_corners[i].y;
        float d_err = sqrt(dx_err * dx_err + dy_err * dy_err);
        float dx_mov = corners[i].x - new_corners[i].x;
        float dy_mov = corners[i].y - new_corners[i].y;
        float d_mov = sqrt(dx_mov * dx_mov + dy_mov * dy_mov);
        if ((int)status[i] == 1 && errors[i] < win_size &&
            (int)status_2[i] == 1 && errors_2[i] < win_size &&
            d_err / d_mov < 0.3 && d_err < 1.0) {
            train_points.push_back(corners[i]);
            query_points.push_back(new_corners[i]);
            cv::line(corner_img, corners[i], new_corners[i], cv::Scalar(0, 0, 255), 1, 8, 0);
        }
    }
    std::cout << "Optical flow pairs: " << train_points.size() << "\n";

    for (size_t i = 0; i < good_matches.size(); ++i) {
        train_points.push_back(keypoints_old[good_matches[i].trainIdx].pt);
        query_points.push_back(keypoints_new[good_matches[i].queryIdx].pt);
    }
    std::cout << "Total pairs: " << train_points.size() << "\n";

    // Show and save intermediate images
    cv::flip(old_projection_img.clone(), old_projection_img, 0);
    cv::flip(new_projection_img.clone(), new_projection_img, 0);
    cv::flip(corner_img.clone(), corner_img, 0);
    cv::flip(correct_matches_img.clone(), correct_matches_img, 0);
    std::stringstream corner_img_ss;
    corner_img_ss << "corner_img_" << direction_index << ".png";
    std::stringstream old_projection_img_ss;
    old_projection_img_ss << "old_projection_img_" << direction_index << ".png";
    std::stringstream new_projection_img_ss;
    new_projection_img_ss << "new_projection_img_" << direction_index << ".png";
    std::stringstream correct_matches_img_ss;
    correct_matches_img_ss << "correct_matches_img_" << direction_index << ".png";
    cv::imshow("corner_img", corner_img);
    cv::imwrite(corner_img_ss.str(), corner_img);
    cv::imshow("old_projection_img", old_projection_img);
    cv::imwrite(old_projection_img_ss.str(), old_projection_img);
    cv::imshow("new_projection_img", new_projection_img);
    cv::imwrite(new_projection_img_ss.str(), new_projection_img);
    cv::imshow("correct_matches_img", correct_matches_img);
    cv::imwrite(correct_matches_img_ss.str(), correct_matches_img);
    cv::waitKey(15);
}

void CloudProjection::detect_matches() {
    normalise_colours(old_pcl_ptr_);
    normalise_colours(new_pcl_ptr_);

    if (Configurations::get_instance()->pi_theta_x != 0) {
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        float theta = M_PI * Configurations::get_instance()->pi_theta_x;
        transform(1, 1) = cos(theta);
        transform(1, 2) = -sin(theta);
        transform(2, 1) = sin(theta);
        transform(2, 2) = cos(theta);
        get_matches_by_direction(transform, 1);

        theta = -M_PI * Configurations::get_instance()->pi_theta_x;
        transform(1, 1) = cos(theta);
        transform(1, 2) = -sin(theta);
        transform(2, 1) = sin(theta);
        transform(2, 2) = cos(theta);
        get_matches_by_direction(transform, 2);
    }
    if (Configurations::get_instance()->pi_theta_y != 0) {
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        float theta = M_PI * Configurations::get_instance()->pi_theta_y;
        transform(0, 0) = cos(theta);
        transform(2, 0) = sin(theta);
        transform(0, 2) = -sin(theta);
        transform(2, 2) = cos(theta);
        get_matches_by_direction(transform, 3);

        theta = -M_PI * Configurations::get_instance()->pi_theta_y;
        transform(0, 0) = cos(theta);
        transform(2, 0) = sin(theta);
        transform(0, 2) = -sin(theta);
        transform(2, 2) = cos(theta);
        get_matches_by_direction(transform, 4);
    }
    get_matches_by_direction(Eigen::Matrix4f::Identity(), 0);
    std::cout << "Total 3D matches " << match_train_indices_.size() << "\n";
    for (size_t i = 0; i < match_train_indices_.size(); ++i) {
        pcl::PointXYZRGB nn_old_point = old_pcl_ptr_->points[match_train_indices_[i]];
        pcl::PointXYZRGB nn_new_point = new_pcl_ptr_->points[match_query_indices_[i]];
        if (direction_indices_[i] != -1) {
            old_parts_ptr_->points.push_back(nn_old_point);
            new_parts_ptr_->points.push_back(nn_new_point);
        }
    }
    old_parts_ptr_->width = old_parts_ptr_->points.size();
    new_parts_ptr_->height = 1;
}
