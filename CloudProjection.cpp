#include "CloudProjection.h"

CloudProjection::CloudProjection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_pcl,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_pcl,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_parts,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_parts) {

    this->p_old_pcl = p_old_pcl;
    this->p_new_pcl = p_new_pcl;
    this->p_old_parts = p_old_parts;
    this->p_new_parts = p_new_parts;
    match_train_indices.clear();
    match_query_indices.clear();
    direction_indices.clear();
}

CloudProjection::~CloudProjection() {

}

void CloudProjection::get_matches_by_direction(Eigen::Matrix4f transform, int direction_index) {

    // Rotate both pointclouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_rotated_old_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_rotated_new_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud (*p_old_pcl, *p_rotated_old_pcl, transform);
    pcl::transformPointCloud (*p_new_pcl, *p_rotated_new_pcl, transform);

    // Estimate normal for rotated pointclouds
    pcl::PointCloud<pcl::Normal>::Ptr p_rotated_old_normal(new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::Normal>::Ptr p_rotated_new_normal(new pcl::PointCloud<pcl::Normal>());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(20);
    norm_est.setInputCloud(p_rotated_old_pcl);
    norm_est.compute(*p_rotated_old_normal);
    std::cout << "Rotated old pointcloud normal size:" << p_rotated_old_normal->points.size() << "\n";
    norm_est.setInputCloud(p_rotated_new_pcl);
    norm_est.compute(*p_rotated_new_normal);
    std::cout << "Rotated new pointcloud normal size:" << p_rotated_new_normal->points.size() << "\n";

    // Flat the rotated pointclouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_flat(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_flat(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (size_t i = 0; i < p_rotated_old_pcl->points.size(); ++i) {
        pcl::PointXYZRGB tmp = p_rotated_old_pcl->points[i];
        tmp.z = 0;
        p_old_flat->points.push_back(tmp);
    }
    p_old_flat->width = p_old_flat->points.size();
    p_old_flat->height = 1;
    p_old_flat->is_dense = 1;
    std::cout << "p_old_flat size: " << p_old_flat->points.size() << "\n";
    for (size_t i = 0; i < p_rotated_new_pcl->points.size(); ++i) {
        pcl::PointXYZRGB tmp = p_rotated_new_pcl->points[i];
        tmp.z = 0;
        p_new_flat->points.push_back(tmp);
    }
    p_new_flat->width = p_new_flat->points.size();
    p_new_flat->height = 1;
    p_new_flat->is_dense = 1;
    std::cout << "p_new_flat size: " << p_new_flat->points.size() << "\n";

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_old_flat;
    kd_old_flat.setInputCloud(p_old_flat);
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_new_flat;
    kd_new_flat.setInputCloud(p_new_flat);

    // Determine pointcloud's boundary
    double x_min = 1e10, y_min = 1e10, z_min = 1e10;
    double x_max = -1e10, y_max = -1e10, z_max = -1e10;
    for (size_t i = 0; i < p_rotated_old_pcl->points.size(); ++i) {
        if (p_rotated_old_pcl->points[i].x < x_min) x_min = p_rotated_old_pcl->points[i].x;
        if (p_rotated_old_pcl->points[i].y < y_min) y_min = p_rotated_old_pcl->points[i].y;
        if (p_rotated_old_pcl->points[i].z < z_min) z_min = p_rotated_old_pcl->points[i].z;
        if (p_rotated_old_pcl->points[i].x > x_max) x_max = p_rotated_old_pcl->points[i].x;
        if (p_rotated_old_pcl->points[i].y > y_max) y_max = p_rotated_old_pcl->points[i].y;
        if (p_rotated_old_pcl->points[i].z > z_max) z_max = p_rotated_old_pcl->points[i].z;
    }
    for (size_t i = 0; i < p_rotated_new_pcl->points.size(); ++i) {
        if (p_rotated_new_pcl->points[i].x < x_min) x_min = p_rotated_new_pcl->points[i].x;
        if (p_rotated_new_pcl->points[i].y < y_min) y_min = p_rotated_new_pcl->points[i].y;
        if (p_rotated_new_pcl->points[i].z < z_min) z_min = p_rotated_new_pcl->points[i].z;
        if (p_rotated_new_pcl->points[i].x > x_max) x_max = p_rotated_new_pcl->points[i].x;
        if (p_rotated_new_pcl->points[i].y > y_max) y_max = p_rotated_new_pcl->points[i].y;
        if (p_rotated_new_pcl->points[i].z > z_max) z_max = p_rotated_new_pcl->points[i].z;
    }
    std::cout << "x: " << x_min << " ~ " << x_max << "\n";
    std::cout << "y: " << y_min << " ~ " << y_max << "\n";
    std::cout << "z: " << z_min << " ~ " << z_max << "\n";
    double dx = x_max - x_min;
    double dy = y_max - y_min;
    double d_min = std::min(dx, dy);
    std::cout << "d_min: " << d_min << "\n";

    // Find image's size
    int low_size = 0;
    int high_size = 1500;
    for (int l = 0; l < 10; l++){
        int picked_size = (low_size + high_size)/2;
        double picked_threshold = d_min/picked_size;
        int picked_x_size = floor(dx/picked_threshold) + 1;
        int picked_y_size = floor(dy/picked_threshold) + 1;
        cv::Mat count_Mat = cv::Mat::zeros(picked_y_size, picked_x_size, CV_32SC1);
        for (int i = 0; i < count_Mat.rows; ++i) {
            for (int j = 0; j < count_Mat.cols; ++j) {
                pcl::PointXYZRGB tmp;
                tmp.x = x_min + j*picked_threshold + picked_threshold/2;
                tmp.y = y_min + i*picked_threshold + picked_threshold/2;
                tmp.z = 0;
                std::vector<int> nn_index;
                std::vector<float> nn_sqd_distance;
                if (!(kd_old_flat.radiusSearch(tmp, 0.5*picked_threshold, nn_index, nn_sqd_distance) > 0)) {
                    continue;
                }
                if (nn_index.size()) {
                    count_Mat.at<int>(i, j)++;
                }
            }
        }
        int single_count = cv::countNonZero(count_Mat);

        count_Mat = cv::Mat::zeros(picked_y_size, picked_x_size, CV_32SC1);
        for (int i = 0; i < count_Mat.rows; ++i) {
            for (int j = 0; j < count_Mat.cols; ++j) {
                pcl::PointXYZRGB tmp;
                tmp.x = x_min + j*picked_threshold + picked_threshold/2;
                tmp.y = y_min + i*picked_threshold + picked_threshold/2;
                tmp.z = 0;
                std::vector<int> nn_index;
                std::vector<float> nn_sqd_distance;
                if (!(kd_old_flat.radiusSearch(tmp, 0.707*picked_threshold, nn_index, nn_sqd_distance) > 0)) {
                    continue;
                }
                if (nn_index.size()) {
                    count_Mat.at<int>(i, j)++;
                }
            }
        }
        int double_count = cv::countNonZero(count_Mat);
        float dense_ratio = (float)single_count/double_count;
        std::cout << dense_ratio << std::endl;
        if (dense_ratio < 0.85) {
            high_size = picked_size;
        }
        else {
            low_size = picked_size;
        }
    }
    std::cout << "low high " << low_size << " " << high_size << "\n";
    if (low_size < 500) return;
    double distance_threshold = d_min / low_size;
    int x_size = floor(dx/distance_threshold) + 1;
    int y_size = floor(dy/distance_threshold) + 1;
    std::cout << "image size: " << x_size << "x" << y_size << "\n";

    // Project old pointcloud
    cv::Mat old_project = cv::Mat::zeros(y_size, x_size, CV_8UC3);
    for (int i = 0; i < old_project.rows; ++i) {
        for (int j = 0; j < old_project.cols; ++j) {
            pcl::PointXYZRGB tmp;
            tmp.x = x_min + j*distance_threshold + distance_threshold/2;
            tmp.y = y_min + i*distance_threshold + distance_threshold/2;
            tmp.z = 0;
            std::vector<int> nn_index;
            std::vector<float> nn_sqd_distance;
            if (!(kd_old_flat.radiusSearch(tmp, distance_threshold, nn_index, nn_sqd_distance) > 0)) {
                continue;
            }
            pcl::PointXYZRGB nn_point = p_rotated_old_pcl->points[nn_index[0]];
            double max_z = -1e10;
            for (size_t k = 0; k < nn_index.size(); ++k) {
                if (p_rotated_old_pcl->points[nn_index[k]].z > max_z) {
                    max_z = p_rotated_old_pcl->points[nn_index[k]].z;
                    nn_point = p_rotated_old_pcl->points[nn_index[k]];
                }
            }
            old_project.at<cv::Vec3b>(i, j)[0] = nn_point.b;
            old_project.at<cv::Vec3b>(i, j)[1] = nn_point.g;
            old_project.at<cv::Vec3b>(i, j)[2] = nn_point.r;
        }
    }

    // Project new pointcloud
    cv::Mat new_project = cv::Mat::zeros(y_size, x_size, CV_8UC3);
    for (int i = 0; i < new_project.rows; ++i) {
        for (int j = 0; j < new_project.cols; ++j) {
            pcl::PointXYZRGB tmp;
            tmp.x = x_min + j*distance_threshold + distance_threshold/2;
            tmp.y = y_min + i*distance_threshold + distance_threshold/2;
            tmp.z = 0;
            std::vector<int> nn_index;
            std::vector<float> nn_sqd_distance;
            if (!(kd_new_flat.radiusSearch(tmp, distance_threshold, nn_index, nn_sqd_distance) > 0)) {
                continue;
            }
            pcl::PointXYZRGB nn_point = p_rotated_new_pcl->points[nn_index[0]];
            double max_z = -1e10;
            for (size_t k = 0; k < nn_index.size(); ++k) {
                if (p_rotated_new_pcl->points[nn_index[k]].z > max_z) {
                    max_z = p_rotated_new_pcl->points[nn_index[k]].z;
                    nn_point = p_rotated_new_pcl->points[nn_index[k]];
                }
            }
            new_project.at<cv::Vec3b>(i, j)[0] = nn_point.b;
            new_project.at<cv::Vec3b>(i, j)[1] = nn_point.g;
            new_project.at<cv::Vec3b>(i, j)[2] = nn_point.r;
        }
    }

    std::vector<cv::Point2f> trainPoints, queryPoints;
    get_2d_matches(old_project, new_project, distance_threshold, trainPoints, queryPoints, direction_index);

    // Get 3d pairs
    std::cout << "OF 2d: " << trainPoints[0] << " " << queryPoints[0] << "\n";
    for (size_t i = 0; i < trainPoints.size(); ++i) {
        cv::Point2f trainPoint = trainPoints[i];
        cv::Point2f queryPoint = queryPoints[i];
        pcl::PointXYZRGB tmp;
        tmp.x = x_min + trainPoint.x*distance_threshold + distance_threshold/2;
        tmp.y = y_min + trainPoint.y*distance_threshold + distance_threshold/2;
        tmp.z = 0;
        std::vector<int> nn_index;
        std::vector<float> nn_sqd_distance;
        if (!(kd_old_flat.radiusSearch(tmp, distance_threshold, nn_index, nn_sqd_distance) > 0)) {
            continue;
        }
        double max_z = -1e10;
        size_t old_best_nn_index = 0;
        pcl::PointXYZRGB nn_old_point = p_rotated_old_pcl->points[nn_index[0]];
        for (size_t j = 0; j < nn_index.size(); ++j) {
            if (p_rotated_old_pcl->points[nn_index[j]].z > max_z) {
                max_z = p_rotated_old_pcl->points[nn_index[j]].z;
                nn_old_point = p_rotated_old_pcl->points[nn_index[j]];
                old_best_nn_index = nn_index[j];
            }
        }
        tmp.x = x_min + queryPoint.x*distance_threshold + distance_threshold/2;
        tmp.y = y_min + queryPoint.y*distance_threshold + distance_threshold/2;
        if (!(kd_new_flat.radiusSearch(tmp, distance_threshold, nn_index, nn_sqd_distance) > 0)) {
            continue;
        }
        max_z = -1e10;
        size_t new_best_nn_index = 0;
        pcl::PointXYZRGB nn_new_point = p_rotated_new_pcl->points[nn_index[0]];
        for (size_t j = 0; j < nn_index.size(); ++j) {
            if (p_rotated_new_pcl->points[nn_index[j]].z > max_z) {
                max_z = p_rotated_new_pcl->points[nn_index[j]].z;
                nn_new_point = p_rotated_new_pcl->points[nn_index[j]];
                new_best_nn_index = nn_index[j];
            }
        }

        double dx = nn_old_point.x - nn_new_point.x;
        double dy = nn_old_point.y - nn_new_point.y;
        double dz = nn_old_point.z - nn_new_point.z;
        float length = sqrt(dx*dx + dy*dy + dz*dz);
        if (length > Configurations::getInstance()->pos_radius) {
            continue;
        }
        match_train_indices.push_back(old_best_nn_index);
        match_query_indices.push_back(new_best_nn_index);
        pcl::Normal old_normal = p_rotated_old_normal->points[old_best_nn_index];
        float old_normal_xy = sqrt(old_normal.normal_x*old_normal.normal_x + old_normal.normal_y*old_normal.normal_y);
        float old_normal_z = fabs(old_normal.normal_z);
        pcl::Normal new_normal = p_rotated_new_normal->points[new_best_nn_index];
        float new_normal_xy = sqrt(new_normal.normal_x*new_normal.normal_x + new_normal.normal_y*new_normal.normal_y);
        float new_normal_z = fabs(new_normal.normal_z);
        if (old_normal_z/old_normal_xy > 0.5 &&
            new_normal_z/new_normal_xy > 0.5) {
            direction_indices.push_back(direction_index);
        }
        else {
            direction_indices.push_back(10);
        }
    }
}

void CloudProjection::get_2d_matches(cv::Mat old_project, cv::Mat new_project, double distance_threshold,
    std::vector<cv::Point2f>& trainPoints, std::vector<cv::Point2f>& queryPoints, int direction_index) {

    //Detect key points using SIFT
    cv::SiftFeatureDetector detector;
    std::vector<cv::KeyPoint> keypoints_old, keypoints_new;
    detector.detect(old_project, keypoints_old);
    std::cout << "keypoints_old's size " << keypoints_old.size() << "\n";
    detector.detect(new_project, keypoints_new);
    std::cout << "keypoints_new's size " << keypoints_new.size() << "\n";

    //Compute descriptors using SIFT
    cv::SiftDescriptorExtractor extractor;
    cv::Mat descriptors_old, descriptors_new;
    extractor.compute(old_project, keypoints_old, descriptors_old);
    extractor.compute(new_project, keypoints_new, descriptors_new);
    int win_size = Configurations::getInstance()->pos_radius/distance_threshold;

    cv::FlannBasedMatcher matcher;
    std::vector<std::vector<cv::DMatch> > matches;
	matcher.knnMatch(descriptors_new, descriptors_old, matches, 2);
    std::vector<cv::DMatch> good_matches;
	for (size_t i = 0; i < matches.size(); i++) {
        if (matches[i][0].distance / matches[i][1].distance < 0.7) {
            cv::Point2f trainPoint = keypoints_old[matches[i][0].trainIdx].pt;
            cv::Point2f queryPoint = keypoints_new[matches[i][0].queryIdx].pt;
            float dx = trainPoint.x - queryPoint.x;
            float dy = trainPoint.y - queryPoint.y;
            float d = sqrt(dx*dx + dy*dy);
            if (d < win_size) {
                good_matches.push_back(matches[i][0]);
            }
        }
    }
    std::cout << "good_matches.size() " << good_matches.size() << "\n";
    cv::Mat img_correctMatches;
    drawMatches(new_project, keypoints_new, old_project, keypoints_old,
        good_matches, img_correctMatches, cv::Scalar::all(-1), cv::Scalar::all(-1),
               std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    // Detect good features to track
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01);
    cv::Size subPixWinSize(1, 1);
    cv::Mat old_project_gray, new_project_gray;
    std::vector<cv::Point2f> corners;
    cv::cvtColor(old_project, old_project_gray, CV_BGR2GRAY);
    cv::cvtColor(new_project, new_project_gray, CV_BGR2GRAY);
    cv::goodFeaturesToTrack(old_project_gray, corners, 50000, 0.01, win_size/2, cv::Mat(), 3, false, 0.04);

    if (corners.size() > 0) {
        cornerSubPix(old_project_gray, corners, subPixWinSize, cv::Size(-1, -1), termcrit);
    }
    std::cout << "corners.size() " << corners.size() << "\n";

    cv::Mat cornerMat = old_project.clone();
    for (size_t i = 0; i < corners.size(); ++i) {
        cv::circle(cornerMat, corners[i], 3, cv::Scalar(255), -1, 8);
    }

    // Execute optical flow
    std::vector<cv::Point2f> new_corners, tmp_corners;
    std::vector<uchar> status, status_2;
    std::vector<float> errors, errors_2;
    cv::Size winSize(win_size, win_size);
    cv::calcOpticalFlowPyrLK(old_project_gray, new_project_gray, corners, new_corners,
        status, errors, winSize, 0, termcrit, 0, 0.0001);
    if (new_corners.size() > 0) {
        cornerSubPix(new_project_gray, new_corners, subPixWinSize, cv::Size(-1, -1), termcrit);
    }
    cv::calcOpticalFlowPyrLK(new_project_gray, old_project_gray, new_corners, tmp_corners,
            status_2, errors_2, winSize, 0, termcrit, 0, 0.0001);
    if (tmp_corners.size() > 0) {
        cornerSubPix(old_project_gray, tmp_corners, subPixWinSize, cv::Size(-1, -1), termcrit);
    }
    float OF_error_threshold = Configurations::getInstance()->OF_error_threshold;
    for (size_t i = 0; i < corners.size(); ++i) {
        float dx_err = corners[i].x - tmp_corners[i].x;
        float dy_err = corners[i].y - tmp_corners[i].y;
        float d_err = sqrt(dx_err*dx_err + dy_err*dy_err);
        float dx_mov = corners[i].x - new_corners[i].x;
        float dy_mov = corners[i].y - new_corners[i].y;
        float d_mov = sqrt(dx_mov*dx_mov + dy_mov*dy_mov);
        if ((int)status[i] == 1 && errors[i] < win_size &&
            (int)status_2[i] == 1 && errors_2[i] < win_size &&
            d_err/d_mov < 0.3 && d_err < 1.0) {
            trainPoints.push_back(corners[i]);
            queryPoints.push_back(new_corners[i]);
            cv::line(cornerMat, corners[i], new_corners[i], cv::Scalar(0, 0, 255), 1, 8, 0);
        }
    }
    std::cout << "Optical flow pairs: " << trainPoints.size() << "\n";

    for (size_t i = 0; i < good_matches.size(); ++i) {
        //trainPoints.push_back(keypoints_old[good_matches[i].trainIdx].pt);
       // queryPoints.push_back(keypoints_new[good_matches[i].queryIdx].pt);
    }
    std::cout << "Total pairs: " << trainPoints.size() << "\n";
    cv::flip(old_project.clone(), old_project, 0);
    cv::flip(new_project.clone(), new_project, 0);
    cv::flip(cornerMat.clone(), cornerMat, 0);
    cv::flip(img_correctMatches.clone(), img_correctMatches, 0);
    std::stringstream ss_cornerMat;
    ss_cornerMat << "cornerMat_" << direction_index << ".png";
    std::stringstream ss_old_project;
    ss_old_project << "old_project_" << direction_index << ".png";
    std::stringstream ss_new_project;
    ss_new_project << "new_project_" << direction_index << ".png";
    std::stringstream ss_correctMatches;
    ss_correctMatches << "correctMatches_" <<  direction_index << ".png";
    cv::imshow("cornerMat", cornerMat);
    cv::imwrite(ss_cornerMat.str(), cornerMat);
    cv::imshow("old_project", old_project);
    cv::imwrite(ss_old_project.str(), old_project);
    cv::imshow("new_project", new_project);
    cv::imwrite(ss_new_project.str(), new_project);
    cv::imshow("img_correctMatches", img_correctMatches);
    cv::imwrite(ss_correctMatches.str(), img_correctMatches);
    cv::waitKey(15);
}

void CloudProjection::draw_matches() {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_pcl_in(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_pcl_in(new pcl::PointCloud<pcl::PointXYZRGB>());
    // Down sampling
    pcl::VoxelGrid<pcl::PointXYZRGB> grid;
    double leaf_size = Configurations::getInstance()->leaf_size*2;
    grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    grid.setInputCloud(p_old_pcl);
    grid.filter(*p_old_pcl_in);
    grid.setInputCloud(p_new_pcl);
    grid.filter(*p_new_pcl_in);

    std::cout << "Draw matches\n";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_matches(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (size_t i = 0; i < p_old_pcl_in->points.size(); ++i) {
        pcl::PointXYZRGB tmp;
        if (Configurations::getInstance()->draw_old_colour) {
            tmp.r = p_old_pcl_in->points[i].r;
            tmp.g = p_old_pcl_in->points[i].g;
            tmp.b = p_old_pcl_in->points[i].b;
        }
        else {
            tmp.r = 0;
            tmp.g = 0;
            tmp.b = 255;
        }
        tmp.x = p_old_pcl_in->points[i].x;
        tmp.y = p_old_pcl_in->points[i].y;
        tmp.z = p_old_pcl_in->points[i].z;
        p_matches->points.push_back(tmp);
    }
    for (size_t i = 0; i < p_new_pcl_in->points.size(); ++i) {
        pcl::PointXYZRGB tmp;
        if (Configurations::getInstance()->draw_new_colour) {
            tmp.r = p_new_pcl_in->points[i].r;
            tmp.g = p_new_pcl_in->points[i].g;
            tmp.b = p_new_pcl_in->points[i].b;
        }
        else {
            tmp.r = 255;
            tmp.g = 0;
            tmp.b = 0;
        }
        tmp.x = p_new_pcl_in->points[i].x;
        tmp.y = p_new_pcl_in->points[i].y;
        tmp.z = p_new_pcl_in->points[i].z;
        p_matches->points.push_back(tmp);
    }
    for (size_t i = 0; i < match_train_indices.size(); ++i) {
        pcl::PointXYZRGB nn_old_point = p_old_pcl->points[match_train_indices[i]];
        pcl::PointXYZRGB nn_new_point = p_new_pcl->points[match_query_indices[i]];
        pcl::PointXYZRGB vec;
        vec.x = nn_old_point.x - nn_new_point.x;
        vec.y = nn_old_point.y - nn_new_point.y;
        vec.z = nn_old_point.z - nn_new_point.z;
        float length = sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
        vec.x /= length;
        vec.y /= length;
        vec.z /= length;
        for (float t = 0; t < 1e10; t += leaf_size / 20) {
            if (t > length) {
                break;
            }
            pcl::PointXYZRGB tran;
            switch (direction_indices[i]) {
                case 0: {
                    tran.r = 0;
                    tran.g = 255;
                    tran.b = 0;
                } break;
                case 1: {
                    tran.r = 255;
                    tran.g = 255;
                    tran.b = 0;
                } break;
                case 2: {
                    tran.r = 0;
                    tran.g = 255;
                    tran.b = 255;
                } break;
                case 3: {
                    tran.r = 255;
                    tran.g = 0;
                    tran.b = 255;
                } break;
                case 4: {
                    tran.r = 255;
                    tran.g = 255;
                    tran.b = 255;
                } break;
                case 10: {
                    tran.r = 0;
                    tran.g = 0;
                    tran.b = 0;
                } break;
            }
            tran.x = nn_new_point.x + t * vec.x;
            tran.y = nn_new_point.y + t * vec.y;
            tran.z = nn_new_point.z + t * vec.z;
            p_matches->points.push_back(tran);
        }
    }
    p_matches->width = p_matches->points.size();
    p_matches->height = 1;
    p_matches->is_dense = 1;
    pcl::io::savePLYFile("Matches.ply", *p_matches, true);
    std::cout << "Matches saved.\n";
}

void CloudProjection::detect_matches() {

    pcl::io::savePLYFile("p_old_pcl.ply", *p_old_pcl, true);
    pcl::io::savePLYFile("p_new_pcl.ply", *p_new_pcl, true);
    draw_matches();

    if (Configurations::getInstance()->pi_theta_x != 0) {
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        float theta = M_PI*Configurations::getInstance()->pi_theta_x;
        transform(1,1) = cos(theta);
        transform(1,2) = -sin(theta);
        transform(2,1) = sin(theta);
        transform(2,2) = cos(theta);
        get_matches_by_direction(transform, 1);

        theta = -M_PI*Configurations::getInstance()->pi_theta_x;
        transform(1,1) = cos(theta);
        transform(1,2) = -sin(theta);
        transform(2,1) = sin(theta);
        transform(2,2) = cos(theta);
        get_matches_by_direction(transform, 2);
    }
    if (Configurations::getInstance()->pi_theta_y != 0) {
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        float theta = M_PI*Configurations::getInstance()->pi_theta_y;
        transform(0,0) = cos(theta);
        transform(2,0) = sin(theta);
        transform(0,2) = -sin(theta);
        transform(2,2) = cos(theta);
        get_matches_by_direction(transform, 3);

        theta = -M_PI*Configurations::getInstance()->pi_theta_y;
        transform(0,0) = cos(theta);
        transform(2,0) = sin(theta);
        transform(0,2) = -sin(theta);
        transform(2,2) = cos(theta);
        get_matches_by_direction(transform, 4);
    }
    get_matches_by_direction(Eigen::Matrix4f::Identity(), 0);
    std::cout << "Total 3D matches " << match_train_indices.size() << "\n";
    for (size_t i = 0; i < match_train_indices.size(); ++i) {
        pcl::PointXYZRGB nn_old_point = p_old_pcl->points[match_train_indices[i]];
        pcl::PointXYZRGB nn_new_point = p_new_pcl->points[match_query_indices[i]];
        if (direction_indices[i] != 10) {
            p_old_parts->points.push_back(nn_old_point);
            p_new_parts->points.push_back(nn_new_point);
        }
    }
    p_old_parts->width = p_old_parts->points.size();
    p_new_parts->height = 1;
    draw_matches();
}