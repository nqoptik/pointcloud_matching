#include "pointcloud_matching/CloudDiffChecker.h"

CloudDiffChecker::CloudDiffChecker(pcl::PointCloud<pcl::PointXYZ>::Ptr p_old_pcl,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr p_new_pcl,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr p_old_parts,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr p_new_parts,
                                   char* matching_results_file) {
    this->pOld = p_old_pcl;
    this->pNew = p_new_pcl;
    this->p_old_parts = p_old_parts;
    this->p_new_parts = p_new_parts;
    this->matching_results_file = matching_results_file;
    pOldDiff = (pcl::PointCloud<pcl::PointXYZ>::Ptr)(new pcl::PointCloud<pcl::PointXYZ>());
    pNewDiff = (pcl::PointCloud<pcl::PointXYZ>::Ptr)(new pcl::PointCloud<pcl::PointXYZ>());
    old_res = 1.0;
    new_res = 1.0;
    min_moving_distance = 100.0f;
    x_min = 1e10;
    y_min = 1e10;
    z_min = 1e10;
    x_max = -1e10;
    y_max = -1e10;
    z_max = -1e10;
    grid_count = 100;
    x_grid_count = 1;
    y_grid_count = 1;
    min_points_in_grid = 15;
    ransacDistanceThreshold = 30;
    getCloudParameters();
    determineDiffRegions();
    griddingDiff();
    getReferPlane();
    getProjections();
    drawTransformation();
}

CloudDiffChecker::~CloudDiffChecker() {
}

float CloudDiffChecker::distance(pcl::PointXYZ p1, pcl::PointXYZ p2) {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    float dz = p1.z - p2.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

float CloudDiffChecker::squaredDistance(pcl::PointXYZ p1, pcl::PointXYZ p2) {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    float dz = p1.z - p2.z;
    return (dx * dx + dy * dy + dz * dz);
}

pcl::PointXYZ CloudDiffChecker::projectionOntoPlane(pcl::PointXYZ project_point, pcl::PointXYZ plane_point, pcl::PointXYZ plane_normal) {
    float t_upper = plane_normal.x * (plane_point.x - project_point.x) + plane_normal.y * (plane_point.y - project_point.y) + plane_normal.z * (plane_point.z - project_point.z);
    float t_lower = plane_normal.x * plane_normal.x + plane_normal.y * plane_normal.y + plane_normal.z * plane_normal.z;
    float t = t_upper / t_lower;
    pcl::PointXYZ point;
    point.x = project_point.x + t * plane_normal.x;
    point.y = project_point.y + t * plane_normal.y;
    point.z = project_point.z + t * plane_normal.z;
    return point;
}

pcl::PointXYZ CloudDiffChecker::lineOntoPlane(pcl::PointXYZ point, pcl::PointXYZ normal, float A, float B, float C, float D) {
    float t_upper = -(A * point.x + B * point.y + C * point.z + D);
    float t_lower = A * normal.x + B * normal.y + C * normal.z;
    float t = t_upper / t_lower;
    pcl::PointXYZ intersec;
    intersec.x = point.x + t * normal.x;
    intersec.y = point.y + t * normal.y;
    intersec.z = point.z + t * normal.z;
    return intersec;
}

double CloudDiffChecker::computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) {
    double resolution = 0.0;
    int n_points = 0;
    int n_resolution;
    std::vector<int> indices(2);
    std::vector<float> sqr_distances(2);
    pcl::search::KdTree<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (!pcl_isfinite((*cloud)[i].x)) {
            continue;
        }
        n_resolution = tree.nearestKSearch(i, 2, indices, sqr_distances);
        if (n_resolution == 2) {
            resolution += sqrt(sqr_distances[1]);
            ++n_points;
        }
    }
    if (n_points != 0) {
        resolution /= n_points;
    }
    return resolution;
}

void CloudDiffChecker::determineDiffRegions() {
    (*pOldDiff).resize(0);
    (*pNewDiff).resize(0);
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_New;
    kdtree_New.setInputCloud(pNew);
    for (size_t i = 0; i < (*pOld).points.size(); i++) {
        pcl::PointXYZ searchPoint = (*pOld).points[i];
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        float radius = min_moving_distance;
        if (!(kdtree_New.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)) {
            (*pOldDiff).points.push_back(searchPoint);
        }
    }
    (*pOldDiff).width = (*pOldDiff).points.size();
    (*pOldDiff).height = 1;
    (*pOldDiff).is_dense = false;
    std::cout << "Old diff size: " << (*pOldDiff).points.size() << "\n";

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_Old;
    kdtree_Old.setInputCloud(pOld);
    for (size_t i = 0; i < (*pNew).points.size(); i++) {
        pcl::PointXYZ searchPoint = (*pNew).points[i];
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        float radius = min_moving_distance;
        if (!(kdtree_Old.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)) {
            (*pNewDiff).points.push_back(searchPoint);
        }
    }
    (*pNewDiff).width = (*pNewDiff).points.size();
    (*pNewDiff).height = 1;
    (*pNewDiff).is_dense = false;
    std::cout << "New diff size: " << (*pNewDiff).points.size() << "\n";
}

void CloudDiffChecker::getCloudParameters() {
    old_res = CloudDiffChecker::computeCloudResolution(pOld);
    new_res = CloudDiffChecker::computeCloudResolution(pNew);
    std::cout << "old_res = " << old_res << "\n";
    std::cout << "new_res = " << new_res << "\n";

    for (size_t i = 0; i < (*pOld).points.size(); i++) {
        if ((*pOld).points[i].x < x_min) {
            x_min = (*pOld).points[i].x;
        }
        if ((*pOld).points[i].y < y_min) {
            y_min = (*pOld).points[i].y;
        }
        if ((*pOld).points[i].z < z_min) {
            z_min = (*pOld).points[i].z;
        }
        if ((*pOld).points[i].x > x_max) {
            x_max = (*pOld).points[i].x;
        }
        if ((*pOld).points[i].y > y_max) {
            y_max = (*pOld).points[i].y;
        }
        if ((*pOld).points[i].z > z_max) {
            z_max = (*pOld).points[i].z;
        }
    }
    for (size_t i = 0; i < (*pNew).points.size(); i++) {
        if ((*pNew).points[i].x < x_min) {
            x_min = (*pNew).points[i].x;
        }
        if ((*pNew).points[i].y < y_min) {
            y_min = (*pNew).points[i].y;
        }
        if ((*pNew).points[i].z < z_min) {
            z_min = (*pNew).points[i].z;
        }
        if ((*pNew).points[i].x > x_max) {
            x_max = (*pNew).points[i].x;
        }
        if ((*pNew).points[i].y > y_max) {
            y_max = (*pNew).points[i].y;
        }
        if ((*pNew).points[i].z > z_max) {
            z_max = (*pNew).points[i].z;
        }
    }

    std::cout << "x_min = " << x_min << " | x_max = " << x_max << " | y_min = " << y_min << " | y_max = "
              << y_max << " | z_min = " << z_min << " | z_max = " << z_max << "\n";
    d_max = x_max - x_min;
    if (y_max - y_min > d_max) {
        d_max = y_max - y_min;
    }
    if (z_max - z_min > d_max) {
        d_max = z_max - z_min;
    }
    std::cout << "d_max = " << d_max << "\n";
    min_moving_distance = d_max / 500;
    std::cout << "min_moving_distance = " << min_moving_distance << "\n";
    float d_x = x_max - x_min;
    float d_y = y_max - y_min;
    if (d_x < d_y) {
        grid_step_length = d_x / grid_count;
    } else {
        grid_step_length = d_y / grid_count;
    }
    std::cout << "grid_step_length = " << grid_step_length << "\n";
    x_grid_count = std::ceil(d_x / grid_step_length);
    y_grid_count = std::ceil(d_y / grid_step_length);
    std::cout << "x_grid_count = " << x_grid_count << "\n";
    std::cout << "y_grid_count = " << y_grid_count << "\n";

    ransacDistanceThreshold = grid_step_length / 10;
    std::cout << "ransacDistanceThreshold: " << ransacDistanceThreshold << "\n";
}

void CloudDiffChecker::griddingDiff() {
    std::cout << "griddingDiff.\n";
    std::vector<std::vector<pcl::PointXYZ>> pOldDiff_grid(x_grid_count * y_grid_count, std::vector<pcl::PointXYZ>());
    std::vector<std::vector<pcl::PointXYZ>> pNewDiff_grid(x_grid_count * y_grid_count, std::vector<pcl::PointXYZ>());
    for (size_t i = 0; i < (*pNewDiff).points.size(); i++) {
        int x_grid_index = MIN(std::floor(((*pNewDiff).points[i].x - x_min) / grid_step_length), x_grid_count - 1);
        int y_grid_index = MIN(std::floor(((*pNewDiff).points[i].y - y_min) / grid_step_length), y_grid_count - 1);
        pNewDiff_grid[y_grid_index * x_grid_count + x_grid_index].push_back(
            pcl::PointXYZ((*pNewDiff).points[i].x, (*pNewDiff).points[i].y, (*pNewDiff).points[i].z));
    }
    for (size_t i = 0; i < (*pOldDiff).points.size(); i++) {
        int x_grid_index = MIN(std::floor(((*pOldDiff).points[i].x - x_min) / grid_step_length), x_grid_count - 1);
        int y_grid_index = MIN(std::floor(((*pOldDiff).points[i].y - y_min) / grid_step_length), y_grid_count - 1);
        pOldDiff_grid[y_grid_index * x_grid_count + x_grid_index].push_back(
            pcl::PointXYZ((*pOldDiff).points[i].x, (*pOldDiff).points[i].y, (*pOldDiff).points[i].z));
    }

    cv::Mat grid_visualization = cv::Mat::zeros(y_grid_count, x_grid_count, CV_8UC1);
    for (size_t i = 0; i < pNewDiff_grid.size(); i++) {
        int x_index = i % x_grid_count;
        int y_index = i / x_grid_count;
        if (pNewDiff_grid[i].size() > min_points_in_grid &&
            pOldDiff_grid[i].size() > min_points_in_grid) {
            grid_visualization.at<uchar>(y_index, x_index) = 255;
        }
    }
    std::cout << "griddingDiff done!\n";
    //dilate(grid_visualization, grid_visualization, cv::Mat(), cv::Point(-1, -1), 1, 1, 1);

    /* Get cluster's index */
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(grid_visualization.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    std::cout << "contours.size() = " << contours.size() << "\n";
    cv::Mat contoursMat = cv::Mat(y_grid_count, x_grid_count, CV_8UC1, 255);
    for (size_t i = 0; i < contours.size(); i++) {
        cv::drawContours(contoursMat, contours, i, i, CV_FILLED);
    }

    clusterIndices = std::vector<int>(x_grid_count * y_grid_count, -1);
    for (size_t i = 0; i < pNewDiff_grid.size(); i++) {
        int x_index = i % x_grid_count;
        int y_index = i / x_grid_count;
        if ((int)contoursMat.at<uchar>(y_index, x_index) == 255) {
            clusterIndices[i] = -1;
        } else {
            clusterIndices[i] = (int)contoursMat.at<uchar>(y_index, x_index);
        }
    }

    /* Index point to fit plane */
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pOld_Clouds(contours.size());
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pNew_Clouds(contours.size());
    for (size_t i = 0; i < contours.size(); i++) {
        pOld_Clouds[i] = (pcl::PointCloud<pcl::PointXYZ>::Ptr)(new pcl::PointCloud<pcl::PointXYZ>());
        pNew_Clouds[i] = (pcl::PointCloud<pcl::PointXYZ>::Ptr)(new pcl::PointCloud<pcl::PointXYZ>());
    }

    for (size_t i = 0; i < pNewDiff_grid.size(); i++) {
        if (clusterIndices[i] >= 0) {
            for (size_t j = 0; j < pOldDiff_grid[i].size(); j++) {
                (*(pOld_Clouds[clusterIndices[i]])).points.push_back(pOldDiff_grid[i][j]);
            }
            for (size_t j = 0; j < pNewDiff_grid[i].size(); j++) {
                (*(pNew_Clouds[clusterIndices[i]])).points.push_back(pNewDiff_grid[i][j]);
            }
        }
    }
    for (size_t i = 0; i < contours.size(); i++) {
        pOld_Clouds[i]->width = (*(pOld_Clouds[i])).points.size();
        pOld_Clouds[i]->height = 1;
        pOld_Clouds[i]->is_dense = false;
        pNew_Clouds[i]->width = (*(pNew_Clouds[i])).points.size();
        pNew_Clouds[i]->height = 1;
        pNew_Clouds[i]->is_dense = false;
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pOld_Clouds_resized(contours.size());
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pNew_Clouds_resized(contours.size());
    for (size_t i = 0; i < contours.size(); i++) {
        pOld_Clouds_resized[i] = (pcl::PointCloud<pcl::PointXYZ>::Ptr)(new pcl::PointCloud<pcl::PointXYZ>());
        pNew_Clouds_resized[i] = (pcl::PointCloud<pcl::PointXYZ>::Ptr)(new pcl::PointCloud<pcl::PointXYZ>());
    }
    for (size_t i = 0; i < pNewDiff_grid.size(); i++) {
        if (clusterIndices[i] >= 0) {
            for (size_t j = 0; j < pOldDiff_grid[i].size(); j++) {
                (*(pOld_Clouds_resized[clusterIndices[i]])).points.push_back(pOldDiff_grid[i][j]);
            }
            for (size_t j = 0; j < pNewDiff_grid[i].size(); j++) {
                (*(pNew_Clouds_resized[clusterIndices[i]])).points.push_back(pNewDiff_grid[i][j]);
            }
        }
    }

    for (size_t i = 0; i < contours.size(); i++) {
        if ((*(pOld_Clouds_resized[i])).points.size() > 5000) {
            std::random_shuffle((*(pOld_Clouds_resized[i])).points.begin(), (*(pOld_Clouds_resized[i])).points.end());
            (*(pOld_Clouds_resized[i])).points.resize(5000);
        }
        pOld_Clouds_resized[i]->width = (*(pOld_Clouds_resized[i])).points.size();
        pOld_Clouds_resized[i]->height = 1;
        pOld_Clouds_resized[i]->is_dense = false;
        if ((*(pNew_Clouds_resized[i])).points.size() > 5000) {
            std::random_shuffle((*(pNew_Clouds_resized[i])).points.begin(), (*(pNew_Clouds_resized[i])).points.end());
            (*(pNew_Clouds_resized[i])).points.resize(5000);
        }
        pNew_Clouds_resized[i]->width = (*(pNew_Clouds_resized[i])).points.size();
        pNew_Clouds_resized[i]->height = 1;
        pNew_Clouds_resized[i]->is_dense = false;
    }

    /* Fit old planes */
    planeCoefficients = std::vector<PlaneCoefficients>(contours.size(), PlaneCoefficients());
    for (size_t i = 0; i < contours.size(); i++) {
        std::vector<int> inliers;
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr old_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(pOld_Clouds_resized[i]));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(old_plane);
        ransac.setDistanceThreshold(ransacDistanceThreshold);
        ransac.computeModel();
        Eigen::VectorXf coeff;
        ransac.getModelCoefficients(coeff);
        ransac.getInliers(inliers);
        if (coeff[2] < 0) {
            for (int j = 0; j < 4; j++) {
                coeff[j] = -coeff[j];
            }
        }
        planeCoefficients[i].o_a = coeff[0];
        planeCoefficients[i].o_b = coeff[1];
        planeCoefficients[i].o_c = coeff[2];
        planeCoefficients[i].o_d = coeff[3];
        planeCoefficients[i].o_inliers = inliers.size();
        planeCoefficients[i].o_total = (*(pOld_Clouds_resized[i])).points.size();
    }
    std::cout << "Fit old planes, done!\n";

    /* Fit new planes */
    for (size_t i = 0; i < contours.size(); i++) {
        std::vector<int> inliers;
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr new_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(pNew_Clouds_resized[i]));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(new_plane);
        ransac.setDistanceThreshold(ransacDistanceThreshold);
        ransac.computeModel();
        Eigen::VectorXf coeff;
        ransac.getModelCoefficients(coeff);
        ransac.getInliers(inliers);
        if (coeff[2] < 0) {
            for (int j = 0; j < 4; j++) {
                coeff[j] = -coeff[j];
            }
        }
        planeCoefficients[i].n_a = coeff[0];
        planeCoefficients[i].n_b = coeff[1];
        planeCoefficients[i].n_c = coeff[2];
        planeCoefficients[i].n_d = coeff[3];
        planeCoefficients[i].n_inliers = inliers.size();
        planeCoefficients[i].n_total = (*(pNew_Clouds_resized[i])).points.size();
    }
    std::cout << "Fit new planes, done!\n";
}

void CloudDiffChecker::getReferPlane() {
    std::vector<ReferPlane> referPlanes_dup;
    referPlanes_dup = std::vector<ReferPlane>(x_grid_count * y_grid_count, ReferPlane());
    for (size_t i = 0; i < referPlanes_dup.size(); i++) {
        int x_index = i % x_grid_count;
        int y_index = i / x_grid_count;
        float centroid_x = x_min + x_index * grid_step_length + grid_step_length / 2;
        float centroid_y = y_min + y_index * grid_step_length + grid_step_length / 2;
        float n_centroid_z = -(planeCoefficients[clusterIndices[i]].n_a * centroid_x + planeCoefficients[clusterIndices[i]].n_b * centroid_y + planeCoefficients[clusterIndices[i]].n_d) / planeCoefficients[clusterIndices[i]].n_c;
        float o_centroid_z = -(planeCoefficients[clusterIndices[i]].o_a * centroid_x + planeCoefficients[clusterIndices[i]].o_b * centroid_y + planeCoefficients[clusterIndices[i]].o_d) / planeCoefficients[clusterIndices[i]].o_c;
        float normal_ref_x = planeCoefficients[clusterIndices[i]].n_a + planeCoefficients[clusterIndices[i]].o_a;
        float normal_ref_y = planeCoefficients[clusterIndices[i]].n_b + planeCoefficients[clusterIndices[i]].o_b;
        float normal_ref_z = planeCoefficients[clusterIndices[i]].n_c + planeCoefficients[clusterIndices[i]].o_c;
        float normal_ref_length = sqrt(normal_ref_x * normal_ref_x + normal_ref_y * normal_ref_y + normal_ref_z * normal_ref_z);
        normal_ref_x /= normal_ref_length;
        normal_ref_y /= normal_ref_length;
        normal_ref_z /= normal_ref_length;
        float centroid_z = (n_centroid_z + o_centroid_z) / 2;
        referPlanes_dup[i].r_a = normal_ref_x;
        referPlanes_dup[i].r_b = normal_ref_y;
        referPlanes_dup[i].r_c = normal_ref_z;
        referPlanes_dup[i].r_d = -(normal_ref_x * centroid_x + normal_ref_y * centroid_y + normal_ref_z * centroid_z);
    }
    referPlanes = std::vector<ReferPlane>(planeCoefficients.size(), ReferPlane());
    std::vector<bool> isSetup(planeCoefficients.size(), false);
    for (size_t i = 0; i < referPlanes_dup.size(); i++) {
        int clusterIndex = clusterIndices[i];
        if (clusterIndex >= 0) {
            if (!isSetup[clusterIndex]) {
                referPlanes[clusterIndex].r_a = referPlanes_dup[i].r_a;
                referPlanes[clusterIndex].r_b = referPlanes_dup[i].r_b;
                referPlanes[clusterIndex].r_c = referPlanes_dup[i].r_c;
                referPlanes[clusterIndex].r_d = referPlanes_dup[i].r_d;
                isSetup[clusterIndex] = true;
            }
        }
    }
    std::cout << "Get refer planes, done!\n";
}

void CloudDiffChecker::getProjections() {
    pOldProj = (pcl::PointCloud<pcl::PointXYZ>::Ptr)(new pcl::PointCloud<pcl::PointXYZ>());
    pNewProj = (pcl::PointCloud<pcl::PointXYZ>::Ptr)(new pcl::PointCloud<pcl::PointXYZ>());

    for (size_t i = 0; i < (*pOld).points.size(); i++) {
        int x_grid_index = MIN(std::floor(((*pOld).points[i].x - x_min) / grid_step_length), x_grid_count - 1);
        int y_grid_index = MIN(std::floor(((*pOld).points[i].y - y_min) / grid_step_length), y_grid_count - 1);
        int grid_index = y_grid_index * x_grid_count + x_grid_index;
        int clusterIndex = clusterIndices[grid_index];
        pcl::PointXYZ projection;
        if (clusterIndex >= 0) {
            pcl::PointXYZ point = (*pOld).points[i];
            pcl::PointXYZ normal;
            normal.x = referPlanes[clusterIndex].r_a;
            normal.y = referPlanes[clusterIndex].r_b;
            normal.z = referPlanes[clusterIndex].r_c;
            projection = lineOntoPlane(point, normal, referPlanes[clusterIndex].r_a, referPlanes[clusterIndex].r_b, referPlanes[clusterIndex].r_c, referPlanes[clusterIndex].r_d);
        } else {
            projection.x = -1e10;
            projection.y = -1e10;
            projection.z = -1e10;
        }
        (*pOldProj).points.push_back(projection);
    }
    for (size_t i = 0; i < (*pNew).points.size(); i++) {
        int x_grid_index = MIN(std::floor(((*pNew).points[i].x - x_min) / grid_step_length), x_grid_count - 1);
        int y_grid_index = MIN(std::floor(((*pNew).points[i].y - y_min) / grid_step_length), y_grid_count - 1);
        int grid_index = y_grid_index * x_grid_count + x_grid_index;
        int clusterIndex = clusterIndices[grid_index];
        pcl::PointXYZ projection;
        if (clusterIndex >= 0) {
            pcl::PointXYZ point = (*pNew).points[i];
            pcl::PointXYZ normal;
            normal.x = referPlanes[clusterIndex].r_a;
            normal.y = referPlanes[clusterIndex].r_b;
            normal.z = referPlanes[clusterIndex].r_c;
            projection = lineOntoPlane(point, normal, referPlanes[clusterIndex].r_a, referPlanes[clusterIndex].r_b, referPlanes[clusterIndex].r_c, referPlanes[clusterIndex].r_d);
        } else {
            projection.x = -1e10;
            projection.y = -1e10;
            projection.z = -1e10;
        }
        (*pNewProj).points.push_back(projection);
    }
    pOldProj->width = (*pOldProj).points.size();
    pOldProj->height = 1;
    pOldProj->is_dense = false;
    pNewProj->width = (*pNewProj).points.size();
    pNewProj->height = 1;
    pNewProj->is_dense = false;
    std::cout << "Get projections, done!\n";
}

void CloudDiffChecker::drawTransformation() {
    std::vector<pcl::PointXYZRGB> ply_scene_cloud;
    pcl::PointXYZRGB pclxyzrgb;
    pclxyzrgb.r = 0;
    pclxyzrgb.g = 0;
    pclxyzrgb.b = 255;
    for (size_t i = 0; i < (*pOld).points.size(); i++) {
        pclxyzrgb.x = (*pOld).points[i].x;
        pclxyzrgb.y = (*pOld).points[i].y;
        pclxyzrgb.z = (*pOld).points[i].z;
        ply_scene_cloud.push_back(pclxyzrgb);
    }
    pclxyzrgb.r = 255;
    pclxyzrgb.g = 0;
    pclxyzrgb.b = 0;
    for (size_t i = 0; i < (*pNew).points.size(); i++) {
        pclxyzrgb.x = (*pNew).points[i].x;
        pclxyzrgb.y = (*pNew).points[i].y;
        pclxyzrgb.z = (*pNew).points[i].z;
        ply_scene_cloud.push_back(pclxyzrgb);
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_OldProj;
    kdtree_OldProj.setInputCloud(pOldProj);
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_NewProj;
    kdtree_NewProj.setInputCloud(pNewProj);

    for (size_t i = 0; i < clusterIndices.size(); i++) {
        if (clusterIndices[i] >= 0) {
            int x_index = i % x_grid_count;
            int y_index = i / x_grid_count;
            float centroid_x = x_min + x_index * grid_step_length + grid_step_length / 2;
            float centroid_y = y_min + y_index * grid_step_length + grid_step_length / 2;

            /* Draw transform vectors */
            pclxyzrgb.r = 255;
            pclxyzrgb.g = 255;
            pclxyzrgb.b = 255;
            for (float x = x_min + x_index * grid_step_length; x < x_min + x_index * grid_step_length + grid_step_length; x += grid_step_length / 2) {
                for (float y = y_min + y_index * grid_step_length; y < y_min + y_index * grid_step_length + grid_step_length; y += grid_step_length / 2) {
                    float z = -(referPlanes[clusterIndices[i]].r_a * x + referPlanes[clusterIndices[i]].r_b * y +
                                referPlanes[clusterIndices[i]].r_d) /
                              referPlanes[clusterIndices[i]].r_c;
                    pcl::PointXYZ searchPoint;
                    searchPoint.x = x;
                    searchPoint.y = y;
                    searchPoint.z = z;
                    std::vector<int> pointIdxRadiusSearch;
                    std::vector<float> pointRadiusSquaredDistance;
                    int min_size_proj = 0;
                    float radius = grid_step_length / 2;
                    pcl::PointXYZ old_terminal(0, 0, 0);
                    float old_stdv = 0;
                    if ((kdtree_OldProj.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)) {
                        for (size_t j = 0; j < pointIdxRadiusSearch.size(); j++) {
                            old_terminal.x += (*pOld).points[pointIdxRadiusSearch[j]].x;
                            old_terminal.y += (*pOld).points[pointIdxRadiusSearch[j]].y;
                            old_terminal.z += (*pOld).points[pointIdxRadiusSearch[j]].z;
                        }
                        old_terminal.x /= pointIdxRadiusSearch.size();
                        old_terminal.y /= pointIdxRadiusSearch.size();
                        old_terminal.z /= pointIdxRadiusSearch.size();
                        for (size_t j = 0; j < pointIdxRadiusSearch.size(); j++) {
                            pcl::PointXYZ oldPoint = (*pOld).points[pointIdxRadiusSearch[j]];
                            old_stdv += squaredDistance(old_terminal, oldPoint);
                        }
                        min_size_proj = pointIdxRadiusSearch.size();
                        old_stdv /= pointIdxRadiusSearch.size();
                        old_stdv = sqrt(old_stdv);
                    }
                    pointIdxRadiusSearch.clear();
                    pointRadiusSquaredDistance.clear();
                    pcl::PointXYZ new_terminal(0, 0, 0);
                    float new_stdv = 0;
                    if ((kdtree_NewProj.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)) {
                        for (size_t j = 0; j < pointIdxRadiusSearch.size(); j++) {
                            new_terminal.x += (*pNew).points[pointIdxRadiusSearch[j]].x;
                            new_terminal.y += (*pNew).points[pointIdxRadiusSearch[j]].y;
                            new_terminal.z += (*pNew).points[pointIdxRadiusSearch[j]].z;
                        }
                        new_terminal.x /= pointIdxRadiusSearch.size();
                        new_terminal.y /= pointIdxRadiusSearch.size();
                        new_terminal.z /= pointIdxRadiusSearch.size();
                        for (size_t j = 0; j < pointIdxRadiusSearch.size(); j++) {
                            pcl::PointXYZ newPoint = (*pNew).points[pointIdxRadiusSearch[j]];
                            new_stdv += squaredDistance(new_terminal, newPoint);
                        }
                        new_stdv /= pointIdxRadiusSearch.size();
                        new_stdv = sqrt(new_stdv);
                        if (pointIdxRadiusSearch.size() < min_size_proj) {
                            min_size_proj = pointIdxRadiusSearch.size();
                        }
                    }

                    if (old_terminal.x != 0 && new_terminal.x != 0 && old_stdv < d_max / 250 && new_stdv < d_max / 250 && min_size_proj > 1) {
                        p_old_parts->points.push_back(old_terminal);
                        p_new_parts->points.push_back(new_terminal);
                        pcl::PointXYZ vec;
                        vec.x = new_terminal.x - old_terminal.x;
                        vec.y = new_terminal.y - old_terminal.y;
                        vec.z = new_terminal.z - old_terminal.z;
                        float length = sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
                        vec.x /= length;
                        vec.y /= length;
                        vec.z /= length;
                        for (float t = 0; t < 1e10; t += grid_step_length / 100) {
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
    p_old_parts->width = p_old_parts->points.size();
    p_old_parts->height = 1;
    p_new_parts->width = p_new_parts->points.size();
    p_new_parts->height = 1;
    std::cout << "ply_scene_cloud.size() = " << ply_scene_cloud.size() << "\n";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr trans(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (size_t i = 0; i < ply_scene_cloud.size(); ++i) {
        trans->points.push_back(ply_scene_cloud[i]);
    }
    trans->width = ply_scene_cloud.size();
    trans->height = 1;
    pcl::io::savePLYFile(matching_results_file, *trans, true);
    std::cout << matching_results_file << " saved.\n";
}
