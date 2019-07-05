#include "pointcloud_matching/matching.hpp"

void draw_keypoints(const std::string& file_name,
                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud_ptr,
                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& keypoints_ptr) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr drawn_keypoints_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (size_t i = 0; i < keypoints_ptr->points.size(); ++i) {
        float step = Configurations::get_instance()->leaf_size / 100;
        pcl::PointXYZRGB point_xyzrgb;
        point_xyzrgb.r = 255;
        point_xyzrgb.g = 0;
        point_xyzrgb.b = 0;
        point_xyzrgb.x = keypoints_ptr->points[i].x + step;
        point_xyzrgb.y = keypoints_ptr->points[i].y;
        point_xyzrgb.z = keypoints_ptr->points[i].z;
        drawn_keypoints_ptr->points.push_back(point_xyzrgb);
        point_xyzrgb.x = keypoints_ptr->points[i].x - step;
        point_xyzrgb.y = keypoints_ptr->points[i].y;
        point_xyzrgb.z = keypoints_ptr->points[i].z;
        drawn_keypoints_ptr->points.push_back(point_xyzrgb);
        point_xyzrgb.x = keypoints_ptr->points[i].x;
        point_xyzrgb.y = keypoints_ptr->points[i].y + step;
        point_xyzrgb.z = keypoints_ptr->points[i].z;
        drawn_keypoints_ptr->points.push_back(point_xyzrgb);
        point_xyzrgb.x = keypoints_ptr->points[i].x;
        point_xyzrgb.y = keypoints_ptr->points[i].y - step;
        point_xyzrgb.z = keypoints_ptr->points[i].z;
        drawn_keypoints_ptr->points.push_back(point_xyzrgb);
        point_xyzrgb.x = keypoints_ptr->points[i].x;
        point_xyzrgb.y = keypoints_ptr->points[i].y;
        point_xyzrgb.z = keypoints_ptr->points[i].z + step;
        drawn_keypoints_ptr->points.push_back(point_xyzrgb);
        point_xyzrgb.x = keypoints_ptr->points[i].x;
        point_xyzrgb.y = keypoints_ptr->points[i].y;
        point_xyzrgb.z = keypoints_ptr->points[i].z - step;
        drawn_keypoints_ptr->points.push_back(point_xyzrgb);
    }
    for (size_t i = 0; i < pointcloud_ptr->points.size(); ++i) {
        pcl::PointXYZRGB point_xyzrgb = pointcloud_ptr->points[i];
        drawn_keypoints_ptr->points.push_back(point_xyzrgb);
    }
    drawn_keypoints_ptr->width = drawn_keypoints_ptr->points.size();
    drawn_keypoints_ptr->height = 1;
    pcl::io::savePLYFile(file_name, *drawn_keypoints_ptr, true);
    std::cout << file_name << " saved.\n";
}

void draw_matching_results(const std::string& file_name,
                           const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_pointcloud_ptr,
                           const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_pointcloud_ptr,
                           const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_parts_ptr,
                           const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_parts_ptr) {
    double leaf_size = Configurations::get_instance()->leaf_size * 2;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr matches_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (size_t i = 0; i < old_pointcloud_ptr->points.size(); ++i) {
        pcl::PointXYZRGB point_xyzrgb;
        if (Configurations::get_instance()->draw_old_colour) {
            point_xyzrgb.r = old_pointcloud_ptr->points[i].r;
            point_xyzrgb.g = old_pointcloud_ptr->points[i].g;
            point_xyzrgb.b = old_pointcloud_ptr->points[i].b;
        } else {
            point_xyzrgb.r = 0;
            point_xyzrgb.g = 0;
            point_xyzrgb.b = 255;
        }
        point_xyzrgb.x = old_pointcloud_ptr->points[i].x;
        point_xyzrgb.y = old_pointcloud_ptr->points[i].y;
        point_xyzrgb.z = old_pointcloud_ptr->points[i].z;
        matches_ptr->points.push_back(point_xyzrgb);
    }
    for (size_t i = 0; i < new_pointcloud_ptr->points.size(); ++i) {
        pcl::PointXYZRGB point_xyzrgb;
        if (Configurations::get_instance()->draw_new_colour) {
            point_xyzrgb.r = new_pointcloud_ptr->points[i].r;
            point_xyzrgb.g = new_pointcloud_ptr->points[i].g;
            point_xyzrgb.b = new_pointcloud_ptr->points[i].b;
        } else {
            point_xyzrgb.r = 255;
            point_xyzrgb.g = 0;
            point_xyzrgb.b = 0;
        }
        point_xyzrgb.x = new_pointcloud_ptr->points[i].x;
        point_xyzrgb.y = new_pointcloud_ptr->points[i].y;
        point_xyzrgb.z = new_pointcloud_ptr->points[i].z;
        matches_ptr->points.push_back(point_xyzrgb);
    }
    for (size_t i = 0; i < old_parts_ptr->points.size(); ++i) {
        pcl::PointXYZRGB vec;
        vec.x = new_parts_ptr->points[i].x - old_parts_ptr->points[i].x;
        vec.y = new_parts_ptr->points[i].y - old_parts_ptr->points[i].y;
        vec.z = new_parts_ptr->points[i].z - old_parts_ptr->points[i].z;
        float length = sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
        if (length > 0) {
            vec.x /= length;
            vec.y /= length;
            vec.z /= length;
            for (float t = 0; t < FLT_MAX; t += leaf_size / 100) {
                pcl::PointXYZRGB point_xyzrgb;
                point_xyzrgb.r = 255 * (t / length);
                point_xyzrgb.g = 255;
                point_xyzrgb.b = 255 - 255 * (t / length);
                if (t > length) {
                    break;
                }
                point_xyzrgb.x = old_parts_ptr->points[i].x + t * vec.x;
                point_xyzrgb.y = old_parts_ptr->points[i].y + t * vec.y;
                point_xyzrgb.z = old_parts_ptr->points[i].z + t * vec.z;
                matches_ptr->points.push_back(point_xyzrgb);
            }
        }
    }
    matches_ptr->width = matches_ptr->points.size();
    matches_ptr->height = 1;
    matches_ptr->is_dense = 1;
    pcl::io::savePLYFile(file_name, *matches_ptr, true);
    std::cout << file_name << " saved.\n";
}

// The keypoints detection functions
void iss_detect_keypoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud_ptr,
                          const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& keypoints_ptr,
                          const bool& is_before) {
    printf("ISS keypoint detection.\n");
    double iss_salient_radius;
    double iss_nonmax_radius;
    int iss_min_neighbours;
    if (is_before) {
        iss_salient_radius = Configurations::get_instance()->iss_old_salient_radius;
        iss_nonmax_radius = Configurations::get_instance()->iss_old_nonmax_radius;
        iss_min_neighbours = Configurations::get_instance()->iss_old_min_neighbours;
    } else {
        iss_salient_radius = Configurations::get_instance()->iss_new_salient_radius;
        iss_nonmax_radius = Configurations::get_instance()->iss_new_nonmax_radius;
        iss_min_neighbours = Configurations::get_instance()->iss_new_min_neighbours;
    }

    // Detect the pointcloud's keypoints using the ISS detetector
    pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss_detector;
    iss_detector.setSalientRadius(iss_salient_radius);
    iss_detector.setNonMaxRadius(iss_nonmax_radius);
    iss_detector.setMinNeighbors(iss_min_neighbours);
    iss_detector.setInputCloud(pointcloud_ptr);
    iss_detector.compute(*keypoints_ptr);
}

void susan_detect_keypoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud_ptr,
                            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& keypoints_ptr,
                            const bool& is_before) {
    printf("Susan keypoint detection.\n");
    double susan_radius;
    if (is_before) {
        susan_radius = Configurations::get_instance()->susan_old_radius;
    } else {
        susan_radius = Configurations::get_instance()->susan_new_radius;
    }

    // Detect the pointcloud's keypoints using the SUSAN detetector
    pcl::SUSANKeypoint<pcl::PointXYZRGB, pcl::PointXYZRGB> susan_detector;
    susan_detector.setRadius(susan_radius);
    susan_detector.setInputCloud(pointcloud_ptr);
    susan_detector.compute(*keypoints_ptr);
}

void harris3d_detect_keypoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud_ptr,
                               const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& keypoints_ptr,
                               const bool& is_before) {
    printf("Harris 3d keypoint detection.\n");

    // Detect the pointcloud's keypoints using the Harris3d detetector
    pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI> harris3d;
    double harris3d_radius = Configurations::get_instance()->harris3d_radius;
    harris3d.setRefine(false);
    harris3d.setInputCloud(pointcloud_ptr);
    if (is_before) {
        for (float hariss_octave = 1.0; hariss_octave <= 2.0; hariss_octave += 1.0) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr xyzi_keypoints_ptr(new pcl::PointCloud<pcl::PointXYZI>);
            harris3d.setRadius(harris3d_radius * hariss_octave);
            harris3d.compute(*xyzi_keypoints_ptr);
            std::cout << "xyzi_keypoints_ptr: " << xyzi_keypoints_ptr->points.size() << "\n";
            for (size_t i = 0; i < xyzi_keypoints_ptr->points.size(); ++i) {
                pcl::PointXYZI point_xyzi = xyzi_keypoints_ptr->points[i];
                pcl::PointXYZRGB point_xyzrgb;
                point_xyzrgb.x = point_xyzi.x;
                point_xyzrgb.y = point_xyzi.y;
                point_xyzrgb.z = point_xyzi.z;
                keypoints_ptr->points.push_back(point_xyzrgb);
            }
        }
    } else {
        for (float hariss_octave = 1.0; hariss_octave <= 2.0; hariss_octave += 1.0) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr xyzi_keypoints_ptr(new pcl::PointCloud<pcl::PointXYZI>);
            harris3d.setRadius(harris3d_radius * hariss_octave);
            harris3d.compute(*xyzi_keypoints_ptr);
            std::cout << "xyzi_keypoints_ptr: " << xyzi_keypoints_ptr->points.size() << "\n";
            for (size_t i = 0; i < xyzi_keypoints_ptr->points.size(); ++i) {
                pcl::PointXYZI point_xyzi = xyzi_keypoints_ptr->points[i];
                pcl::PointXYZRGB point_xyzrgb;
                point_xyzrgb.x = point_xyzi.x;
                point_xyzrgb.y = point_xyzi.y;
                point_xyzrgb.z = point_xyzi.z;
                keypoints_ptr->points.push_back(point_xyzrgb);
            }
        }
    }
    keypoints_ptr->width = keypoints_ptr->points.size();
    keypoints_ptr->height = 1;
}

void two_dimension_detect_keypoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_pointcloud_ptr,
                                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_pointcloud_ptr,
                                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_keypoints_ptr,
                                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_keypoints_ptr,
                                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_parts_ptr,
                                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_parts_ptr) {
    printf("2D matching method.\n");
    CloudProjection cloud_projection(old_pointcloud_ptr, new_pointcloud_ptr, old_parts_ptr, new_parts_ptr);
    cloud_projection.detect_matches();
}

// The descriptors extraction functions
void icp_extract_description(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_pointcloud_ptr,
                             const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_pointcloud_ptr,
                             const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_keypoints_ptr,
                             const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_keypoints_ptr,
                             const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_parts_ptr,
                             const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_parts_ptr) {
    printf("ICP matching.\n");

    // Down sample the pointclouds to speedup the icp process
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
    double leaf_size = Configurations::get_instance()->leaf_size * 2.5;
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);

    // Down sample the old pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr old_pointcloud_icp_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    voxel_grid.setInputCloud(old_pointcloud_ptr);
    voxel_grid.filter(*old_pointcloud_icp_ptr);
    std::cout << "old_pointcloud_icp_ptr: " << *old_pointcloud_icp_ptr << "\n";

    // Down sample the new pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_pointcloud_icp_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    voxel_grid.setInputCloud(new_pointcloud_ptr);
    voxel_grid.filter(*new_pointcloud_icp_ptr);
    std::cout << "new_pointcloud_icp_ptr: " << *new_pointcloud_icp_ptr << "\n";

    // Initialise the icp parameters
    double icp_radius = Configurations::get_instance()->icp_radius;
    int icp_iterations = Configurations::get_instance()->icp_iterations;
    double pos_radius = Configurations::get_instance()->pos_radius;
    double icp_refine_radius = Configurations::get_instance()->icp_refine_radius;

    // Initialise the KdTreeFLANN search for the old icp pointcloud
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_tree_flann_old;
    kd_tree_flann_old.setInputCloud(old_pointcloud_icp_ptr);

    // Initialise the KdTreeFLANN search for the new icp pointcloud
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_tree_flann_new;
    kd_tree_flann_new.setInputCloud(new_pointcloud_icp_ptr);

    // Initialise the KdTreeFLANN search for the old keypoint pointcloud
    pcl::KdTreeFLANN<pcl::PointXYZRGB> new_kd_tree_flann_keypoints;
    new_kd_tree_flann_keypoints.setInputCloud(new_keypoints_ptr);

    // Search the most similar point from the possile regions
    for (size_t i = 0; i < old_keypoints_ptr->points.size(); ++i) {
        std::cout << " Matching process: " << i << "/" << old_keypoints_ptr->points.size() << "\n";
        pcl::PointXYZRGB old_keypoint = (old_keypoints_ptr->points)[i];
        std::vector<int> k_indices;
        std::vector<float> k_squared_distances;
        if (!kd_tree_flann_old.radiusSearch(old_keypoint, icp_radius, k_indices, k_squared_distances) > 0) {
            std::cout << " !not found old neighbours\n";
            continue;
        }

        // The old keypoint's neighbours
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr old_neighbours(new pcl::PointCloud<pcl::PointXYZRGB>());
        for (size_t j = 0; j < k_indices.size(); ++j) {
            old_neighbours->points.push_back(old_pointcloud_icp_ptr->points[k_indices[j]]);
        }

        // Get the possible refer keypoints
        std::vector<int> possible_refer_indices;
        std::vector<float> possible_squared_distances;
        if (!new_kd_tree_flann_keypoints.radiusSearch(old_keypoint, pos_radius, possible_refer_indices, possible_squared_distances) > 0) {
            std::cout << " !not found possible refer keypoint.\n";
            continue;
        }
        float best_score = FLT_MAX;
        int best_refer_index = 0;
        for (size_t j = 0; j < possible_refer_indices.size(); ++j) {
            pcl::PointXYZRGB new_keypoint = (new_keypoints_ptr->points)[possible_refer_indices[j]];
            std::vector<int> k_indices;
            std::vector<float> k_squared_distances;
            if (!kd_tree_flann_new.radiusSearch(new_keypoint, icp_radius, k_indices, k_squared_distances) > 0) {
                std::cout << " !not found new neighbours\n";
                continue;
            }

            // The new keypoint's neighbours
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_neighbours(new pcl::PointCloud<pcl::PointXYZRGB>());
            for (size_t k = 0; k < k_indices.size(); ++k) {
                new_neighbours->points.push_back(new_pointcloud_icp_ptr->points[k_indices[k]]);
            }
            pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
            icp.setMaximumIterations(icp_iterations);
            icp.setInputSource(new_neighbours);
            icp.setInputTarget(old_neighbours);
            icp.align(*new_neighbours);
            icp.setMaximumIterations(1);
            icp.align(*new_neighbours);
            if (icp.hasConverged()) {
                if (icp.getFitnessScore() < best_score) {
                    best_score = icp.getFitnessScore();
                    best_refer_index = j;
                    std::cout << " best index: " << best_refer_index << " score: " << best_score << " refer point " << j << "/" << possible_refer_indices.size()
                              << " size icp " << new_neighbours->points.size() << " -> " << old_neighbours->points.size() << "\n";
                }
            }
        }
        if (best_score < FLT_MAX) {
            std::cout << "Refine";
            pcl::PointXYZRGB similar_keypoint = new_keypoints_ptr->points[possible_refer_indices[best_refer_index]];
            std::vector<int> refine_index;
            std::vector<float> refine_sqd;
            if (!kd_tree_flann_new.radiusSearch(similar_keypoint, icp_refine_radius, refine_index, refine_sqd) > 0) {
                std::cout << " !not found refine point\n";
                continue;
            }
            float refine_best_score = FLT_MAX;
            int refine_best_refer_index = 0;
            float diff_size_ratio = 1;
            for (size_t j = 0; j < refine_index.size(); ++j) {
                pcl::PointXYZRGB new_similar_point = (new_pointcloud_icp_ptr->points)[refine_index[j]];
                std::vector<int> k_indices;
                std::vector<float> k_squared_distances;
                if (!kd_tree_flann_new.radiusSearch(new_similar_point, icp_radius, k_indices, k_squared_distances) > 0) {
                    std::cout << " !not found new neighbours\n";
                    continue;
                }

                // The new keypoint's neighbours
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_neighbours(new pcl::PointCloud<pcl::PointXYZRGB>());
                for (size_t k = 0; k < k_indices.size(); ++k) {
                    new_neighbours->points.push_back(new_pointcloud_icp_ptr->points[k_indices[k]]);
                }
                pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
                icp.setMaximumIterations(icp_iterations);
                icp.setInputSource(new_neighbours);
                icp.setInputTarget(old_neighbours);
                icp.align(*new_neighbours);
                icp.setMaximumIterations(1);
                icp.align(*new_neighbours);
                if (icp.hasConverged()) {
                    if (icp.getFitnessScore() < refine_best_score) {
                        refine_best_score = icp.getFitnessScore();
                        refine_best_refer_index = j;
                        std::cout << " precess: " << j << " / " << refine_index.size() << "\n";
                        float old_size = (float)old_neighbours->points.size();
                        float new_size = (float)new_neighbours->points.size();
                        float max_size = std::max(old_size, new_size);
                        float min_size = std::min(old_size, new_size);
                        diff_size_ratio = min_size / max_size;
                    }
                }
            }
            pcl::PointXYZRGB nearest_point = new_pointcloud_icp_ptr->points[refine_index[refine_best_refer_index]];
            pcl::PointXYZRGB old_part = old_keypoint;
            old_part.r = 255 * diff_size_ratio;
            old_part.g = 255 * diff_size_ratio;
            old_part.b = 255 * diff_size_ratio;
            old_parts_ptr->points.push_back(old_part);
            new_parts_ptr->points.push_back(nearest_point);
        }
    }
}

void shot_extract_description(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_pointcloud_ptr,
                              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_pointcloud_ptr,
                              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_keypoints_ptr,
                              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_keypoints_ptr,
                              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_parts_ptr,
                              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_parts_ptr) {
    printf("SHOT matching.\n");

    // Initialise the KdTree search for normal estimation
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
    norm_est.setSearchMethod(kd_tree);
    norm_est.setKSearch(10);

    // Estimation the normals of the old pointcloud
    pcl::PointCloud<pcl::Normal>::Ptr old_normal_ptr(new pcl::PointCloud<pcl::Normal>());
    norm_est.setInputCloud(old_pointcloud_ptr);
    norm_est.compute(*old_normal_ptr);
    std::cout << "old_normal_ptr: " << old_normal_ptr->points.size() << "\n";

    // Estimation the normals of the new pointcloud
    pcl::PointCloud<pcl::Normal>::Ptr new_normal_ptr(new pcl::PointCloud<pcl::Normal>());
    norm_est.setInputCloud(new_pointcloud_ptr);
    norm_est.compute(*new_normal_ptr);
    std::cout << "new_normal_ptr: " << new_normal_ptr->points.size() << "\n";

    // Initialise the SHOT descriptors extrator
    double shot_radius = Configurations::get_instance()->shot_radius;
    pcl::SHOTEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352> descriptors_extractor;
    descriptors_extractor.setRadiusSearch(shot_radius);

    // Extract the SHOT descriptors of the old pointcloud
    pcl::PointCloud<pcl::SHOT352>::Ptr old_shot_ptr(new pcl::PointCloud<pcl::SHOT352>());
    descriptors_extractor.setInputCloud(old_keypoints_ptr);
    descriptors_extractor.setInputNormals(old_normal_ptr);
    descriptors_extractor.setSearchSurface(old_pointcloud_ptr);
    descriptors_extractor.compute(*old_shot_ptr);
    std::cout << "old_shot_ptr: " << old_shot_ptr->points.size() << "\n";

    // Extract the SHOT descriptors of the new pointcloud
    pcl::PointCloud<pcl::SHOT352>::Ptr new_shot_ptr(new pcl::PointCloud<pcl::SHOT352>());
    descriptors_extractor.setInputCloud(new_keypoints_ptr);
    descriptors_extractor.setInputNormals(new_normal_ptr);
    descriptors_extractor.setSearchSurface(new_pointcloud_ptr);
    descriptors_extractor.compute(*new_shot_ptr);
    std::cout << "new_shot_ptr: " << new_shot_ptr->points.size() << "\n";

    // Initialise the KdTreeFLANN search for the old keypoint pointcloud
    pcl::KdTreeFLANN<pcl::PointXYZRGB> old_kd_tree_flann_keypoints;
    old_kd_tree_flann_keypoints.setInputCloud(old_keypoints_ptr);

    // Initialise the KdTreeFLANN search for the new keypoint pointcloud
    pcl::KdTreeFLANN<pcl::PointXYZRGB> new_kd_tree_flann_keypoints;
    new_kd_tree_flann_keypoints.setInputCloud(new_keypoints_ptr);

    // Estimate the threshold
    double pos_radius = Configurations::get_instance()->pos_radius;
    std::vector<float> descriptor_distances;
    for (size_t i = 0; i < old_keypoints_ptr->points.size(); ++i) {
        pcl::PointXYZRGB old_keypoint = old_keypoints_ptr->points[i];
        std::vector<int> possible_refer_indices;
        std::vector<float> possible_squared_distances;
        if (!new_kd_tree_flann_keypoints.radiusSearch(old_keypoint, pos_radius, possible_refer_indices, possible_squared_distances) > 0) {
            continue;
        }
        int new_descriptor_index = -1;
        float min_descriptor_d = FLT_MAX;
        for (size_t j = 0; j < possible_refer_indices.size(); ++j) {
            int new_descriptor_index_ = possible_refer_indices[j];
            float descriptor_d = 0;
            for (int k = 0; k < 352; ++k) {
                descriptor_d += (old_shot_ptr->points[i].descriptor[k] - new_shot_ptr->points[new_descriptor_index_].descriptor[k]) *
                                (old_shot_ptr->points[i].descriptor[k] - new_shot_ptr->points[new_descriptor_index_].descriptor[k]);
            }
            descriptor_d = sqrt(descriptor_d);
            if (descriptor_d < min_descriptor_d) {
                min_descriptor_d = descriptor_d;
                new_descriptor_index = new_descriptor_index_;
            }
        }
        pcl::PointXYZRGB old_part = old_keypoint;
        pcl::PointXYZRGB new_part = new_keypoints_ptr->points[new_descriptor_index];
        if (new_descriptor_index != -1) {
            descriptor_distances.push_back(min_descriptor_d);
        }
    }
    float descriptor_distance_avg = 0;
    for (size_t i = 0; i < descriptor_distances.size(); ++i) {
        descriptor_distance_avg += descriptor_distances[i];
    }
    descriptor_distance_avg /= descriptor_distances.size();
    float descriptor_distance_threshold = descriptor_distance_avg;
    std::cout << "descriptor_distance_threshold: " << descriptor_distance_threshold << "\n";

    // Find the descriptor matches
    for (size_t i = 0; i < old_keypoints_ptr->points.size(); ++i) {
        pcl::PointXYZRGB old_keypoint = old_keypoints_ptr->points[i];
        std::vector<int> possible_refer_indices;
        std::vector<float> possible_squared_distances;
        if (!new_kd_tree_flann_keypoints.radiusSearch(old_keypoint, pos_radius, possible_refer_indices, possible_squared_distances) > 0) {
            continue;
        }
        int new_descriptor_index = -1;
        float min_descriptor_d = FLT_MAX;
        for (size_t j = 0; j < possible_refer_indices.size(); ++j) {
            int new_descriptor_index_ = possible_refer_indices[j];
            float descriptor_d = 0;
            for (int k = 0; k < 352; ++k) {
                descriptor_d += (old_shot_ptr->points[i].descriptor[k] - new_shot_ptr->points[new_descriptor_index_].descriptor[k]) *
                                (old_shot_ptr->points[i].descriptor[k] - new_shot_ptr->points[new_descriptor_index_].descriptor[k]);
            }
            descriptor_d = sqrt(descriptor_d);
            if (descriptor_d < min_descriptor_d) {
                min_descriptor_d = descriptor_d;
                new_descriptor_index = new_descriptor_index_;
            }
        }
        if (min_descriptor_d < descriptor_distance_threshold) {
            pcl::PointXYZRGB new_keypoint = new_keypoints_ptr->points[new_descriptor_index];
            if (!old_kd_tree_flann_keypoints.radiusSearch(new_keypoint, pos_radius, possible_refer_indices, possible_squared_distances) > 0) {
                continue;
            }
            int old_descriptor_index = -1;
            float min_descriptor_d_old = FLT_MAX;
            for (size_t j = 0; j < possible_refer_indices.size(); ++j) {
                int old_descriptor_index_ = possible_refer_indices[j];
                float descriptor_d = 0;
                for (int k = 0; k < 352; ++k) {
                    descriptor_d += (old_shot_ptr->points[old_descriptor_index_].descriptor[k] - new_shot_ptr->points[new_descriptor_index].descriptor[k]) *
                                    (old_shot_ptr->points[old_descriptor_index_].descriptor[k] - new_shot_ptr->points[new_descriptor_index].descriptor[k]);
                }
                descriptor_d = sqrt(descriptor_d);
                if (descriptor_d < min_descriptor_d_old) {
                    min_descriptor_d_old = descriptor_d;
                    old_descriptor_index = old_descriptor_index_;
                }
            }
            if (old_descriptor_index == i) {
                pcl::PointXYZRGB old_part = old_keypoint;
                pcl::PointXYZRGB new_part = new_keypoints_ptr->points[new_descriptor_index];
                old_parts_ptr->points.push_back(old_part);
                new_parts_ptr->points.push_back(new_part);
            }
        }
    }
}

void fpfh_extract_description(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_pointcloud_ptr,
                              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_pointcloud_ptr,
                              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_keypoints_ptr,
                              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_keypoints_ptr,
                              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_parts_ptr,
                              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_parts_ptr) {
    printf("FPFH matching.\n");

    // Initialise the KdTree search for normal estimation
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
    norm_est.setSearchMethod(kd_tree);
    norm_est.setKSearch(10);

    // Estimation the normals of the old pointcloud
    pcl::PointCloud<pcl::Normal>::Ptr old_normal_ptr(new pcl::PointCloud<pcl::Normal>());
    norm_est.setInputCloud(old_pointcloud_ptr);
    norm_est.compute(*old_normal_ptr);
    std::cout << "old_normal_ptr: " << old_normal_ptr->points.size() << "\n";

    // Estimation the normals of the new pointcloud
    pcl::PointCloud<pcl::Normal>::Ptr new_normal_ptr(new pcl::PointCloud<pcl::Normal>());
    norm_est.setInputCloud(new_pointcloud_ptr);
    norm_est.compute(*new_normal_ptr);
    std::cout << "new_normal_ptr: " << new_normal_ptr->points.size() << "\n";

    // Initialise the KdTreeFLANN search for the old pointcloud
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_tree_flann_old;
    kd_tree_flann_old.setInputCloud(old_pointcloud_ptr);

    // Initialise the KdTreeFLANN search for the new pointcloud
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_tree_flann_new;
    kd_tree_flann_new.setInputCloud(new_pointcloud_ptr);

    // Get the keypoints' indices
    boost::shared_ptr<std::vector<int>> old_keypoint_index_in_pcl(new std::vector<int>(0));
    for (size_t i = 0; i < old_keypoints_ptr->points.size(); ++i) {
        pcl::PointXYZRGB old_point = old_keypoints_ptr->points[i];
        std::vector<int> k_indices;
        std::vector<float> k_squared_distances;
        if (!kd_tree_flann_old.nearestKSearch(old_point, 1, k_indices, k_squared_distances) > 0) {
            std::cout << " Check get old keypoint's indices.\n";
            exit(1);
        }
        old_keypoint_index_in_pcl->push_back(k_indices[0]);
    }
    std::cout << "old_keypoint_index_in_pcl: " << old_keypoint_index_in_pcl->size() << "\n";

    boost::shared_ptr<std::vector<int>> new_keypoint_index_in_pcl(new std::vector<int>(0));
    for (size_t i = 0; i < new_keypoints_ptr->points.size(); ++i) {
        pcl::PointXYZRGB new_point = new_keypoints_ptr->points[i];
        std::vector<int> k_indices;
        std::vector<float> k_squared_distances;
        if (!kd_tree_flann_new.nearestKSearch(new_point, 1, k_indices, k_squared_distances) > 0) {
            std::cout << " Check get new keypoint's indices.\n";
            exit(1);
        }
        new_keypoint_index_in_pcl->push_back(k_indices[0]);
    }
    std::cout << "new_keypoint_index_in_pcl: " << new_keypoint_index_in_pcl->size() << "\n";

    // Initialise the FPFH descriptors extrator
    double fpfh_radius = Configurations::get_instance()->fpfh_radius;
    pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> descriptors_extractor;
    norm_est.setSearchMethod(kd_tree);
    descriptors_extractor.setRadiusSearch(fpfh_radius);

    // Extract the FPFH descriptors of the old pointcloud
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr old_fpfh_ptr(new pcl::PointCloud<pcl::FPFHSignature33>());
    descriptors_extractor.setInputCloud(old_pointcloud_ptr);
    descriptors_extractor.setIndices(old_keypoint_index_in_pcl);
    descriptors_extractor.setInputNormals(old_normal_ptr);
    descriptors_extractor.compute(*old_fpfh_ptr);
    std::cout << "old_fpfh_ptr: " << old_fpfh_ptr->points.size() << "\n";

    // Extract the FPFH descriptors of the new pointcloud
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr new_fpfh_ptr(new pcl::PointCloud<pcl::FPFHSignature33>());
    descriptors_extractor.setInputCloud(new_pointcloud_ptr);
    descriptors_extractor.setIndices(new_keypoint_index_in_pcl);
    descriptors_extractor.setInputNormals(new_normal_ptr);
    descriptors_extractor.compute(*new_fpfh_ptr);
    std::cout << "new_fpfh_ptr: " << new_fpfh_ptr->points.size() << "\n";

    // Initialise the KdTreeFLANN search for the old keypoint pointcloud
    pcl::KdTreeFLANN<pcl::PointXYZRGB> old_kd_tree_flann_keypoints;
    old_kd_tree_flann_keypoints.setInputCloud(old_keypoints_ptr);

    // Initialise the KdTreeFLANN search for the new keypoint pointcloud
    pcl::KdTreeFLANN<pcl::PointXYZRGB> new_kd_tree_flann_keypoints;
    new_kd_tree_flann_keypoints.setInputCloud(new_keypoints_ptr);

    // Estimate the threshold
    double pos_radius = Configurations::get_instance()->pos_radius;
    std::vector<float> descriptor_distances;
    for (size_t i = 0; i < old_keypoints_ptr->points.size(); ++i) {
        pcl::PointXYZRGB old_keypoint = old_keypoints_ptr->points[i];
        std::vector<int> possible_refer_indices;
        std::vector<float> possible_squared_distances;
        if (!new_kd_tree_flann_keypoints.radiusSearch(old_keypoint, pos_radius, possible_refer_indices, possible_squared_distances) > 0) {
            continue;
        }
        int new_descriptor_index = -1;
        float min_descriptor_d = FLT_MAX;
        for (size_t j = 0; j < possible_refer_indices.size(); ++j) {
            int new_descriptor_index_ = possible_refer_indices[j];
            float descriptor_d = 0;
            for (int k = 0; k < 33; ++k) {
                descriptor_d += (old_fpfh_ptr->points[i].histogram[k] - new_fpfh_ptr->points[new_descriptor_index_].histogram[k]) *
                                (old_fpfh_ptr->points[i].histogram[k] - new_fpfh_ptr->points[new_descriptor_index_].histogram[k]);
            }
            descriptor_d = sqrt(descriptor_d);
            if (descriptor_d < min_descriptor_d) {
                min_descriptor_d = descriptor_d;
                new_descriptor_index = new_descriptor_index_;
            }
        }
        pcl::PointXYZRGB old_part = old_keypoint;
        pcl::PointXYZRGB new_part = new_keypoints_ptr->points[new_descriptor_index];
        if (new_descriptor_index != -1) {
            descriptor_distances.push_back(min_descriptor_d);
        }
    }
    float descriptor_distance_avg = 0;
    for (size_t i = 0; i < descriptor_distances.size(); ++i) {
        descriptor_distance_avg += descriptor_distances[i];
    }
    descriptor_distance_avg /= descriptor_distances.size();
    float descriptor_distance_threshold = descriptor_distance_avg;
    std::cout << "descriptor_distance_threshold: " << descriptor_distance_threshold << "\n";

    // Find the descriptor matches
    for (size_t i = 0; i < old_keypoints_ptr->points.size(); ++i) {
        pcl::PointXYZRGB old_keypoint = old_keypoints_ptr->points[i];
        std::vector<int> possible_refer_indices;
        std::vector<float> possible_squared_distances;
        if (!new_kd_tree_flann_keypoints.radiusSearch(old_keypoint, pos_radius, possible_refer_indices, possible_squared_distances) > 0) {
            continue;
        }
        int new_descriptor_index = -1;
        float min_descriptor_d = FLT_MAX;
        for (size_t j = 0; j < possible_refer_indices.size(); ++j) {
            int new_descriptor_index_ = possible_refer_indices[j];
            float descriptor_d = 0;
            for (int k = 0; k < 33; ++k) {
                descriptor_d += (old_fpfh_ptr->points[i].histogram[k] - new_fpfh_ptr->points[new_descriptor_index_].histogram[k]) *
                                (old_fpfh_ptr->points[i].histogram[k] - new_fpfh_ptr->points[new_descriptor_index_].histogram[k]);
            }
            descriptor_d = sqrt(descriptor_d);
            if (descriptor_d < min_descriptor_d) {
                min_descriptor_d = descriptor_d;
                new_descriptor_index = new_descriptor_index_;
            }
        }
        if (min_descriptor_d < descriptor_distance_threshold) {
            pcl::PointXYZRGB new_keypoint = new_keypoints_ptr->points[new_descriptor_index];
            if (!old_kd_tree_flann_keypoints.radiusSearch(new_keypoint, pos_radius, possible_refer_indices, possible_squared_distances) > 0) {
                continue;
            }
            int old_descriptor_index = -1;
            float min_descriptor_d_old = FLT_MAX;
            for (size_t j = 0; j < possible_refer_indices.size(); ++j) {
                int old_descriptor_index_ = possible_refer_indices[j];
                float descriptor_d = 0;
                for (int k = 0; k < 33; ++k) {
                    descriptor_d += (old_fpfh_ptr->points[old_descriptor_index_].histogram[k] - new_fpfh_ptr->points[new_descriptor_index].histogram[k]) *
                                    (old_fpfh_ptr->points[old_descriptor_index_].histogram[k] - new_fpfh_ptr->points[new_descriptor_index].histogram[k]);
                }
                descriptor_d = sqrt(descriptor_d);
                if (descriptor_d < min_descriptor_d_old) {
                    min_descriptor_d_old = descriptor_d;
                    old_descriptor_index = old_descriptor_index_;
                }
            }
            if (old_descriptor_index == i) {
                pcl::PointXYZRGB old_part = old_keypoint;
                pcl::PointXYZRGB new_part = new_keypoints_ptr->points[new_descriptor_index];
                old_parts_ptr->points.push_back(old_part);
                new_parts_ptr->points.push_back(new_part);
            }
        }
    }
}

void shotcolor_extract_description(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_pointcloud_ptr,
                                   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_pointcloud_ptr,
                                   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_keypoints_ptr,
                                   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_keypoints_ptr,
                                   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& old_parts_ptr,
                                   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& new_parts_ptr) {
    printf("SHOTCOLOR matching.\n");
    normalise_colours(old_pointcloud_ptr);
    normalise_colours(new_pointcloud_ptr);

    // Initialise the KdTree search for normal estimation
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
    norm_est.setSearchMethod(kd_tree);
    norm_est.setKSearch(10);

    // Estimation the normals of the old pointcloud
    pcl::PointCloud<pcl::Normal>::Ptr old_normal_ptr(new pcl::PointCloud<pcl::Normal>());
    norm_est.setInputCloud(old_pointcloud_ptr);
    norm_est.compute(*old_normal_ptr);
    std::cout << "old_normal_ptr: " << old_normal_ptr->points.size() << "\n";

    // Estimation the normals of the new pointcloud
    pcl::PointCloud<pcl::Normal>::Ptr new_normal_ptr(new pcl::PointCloud<pcl::Normal>());
    norm_est.setInputCloud(new_pointcloud_ptr);
    norm_est.compute(*new_normal_ptr);
    std::cout << "new_normal_ptr: " << new_normal_ptr->points.size() << "\n";

    // Initialise the SHOTCOLOR descriptors extrator
    double shot_radius = Configurations::get_instance()->shot_radius;
    pcl::SHOTColorEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344> descriptors_extractor;
    descriptors_extractor.setRadiusSearch(shot_radius);

    // Extract the SHOTCOLOR descriptors of the old pointcloud
    pcl::PointCloud<pcl::SHOT1344>::Ptr old_shot_ptr(new pcl::PointCloud<pcl::SHOT1344>());
    descriptors_extractor.setInputCloud(old_keypoints_ptr);
    descriptors_extractor.setInputNormals(old_normal_ptr);
    descriptors_extractor.setSearchSurface(old_pointcloud_ptr);
    descriptors_extractor.compute(*old_shot_ptr);
    std::cout << "old_shot_ptr: " << old_shot_ptr->points.size() << "\n";

    // Extract the SHOTCOLOR descriptors of the new pointcloud
    pcl::PointCloud<pcl::SHOT1344>::Ptr new_shot_ptr(new pcl::PointCloud<pcl::SHOT1344>());
    descriptors_extractor.setInputCloud(new_keypoints_ptr);
    descriptors_extractor.setInputNormals(new_normal_ptr);
    descriptors_extractor.setSearchSurface(new_pointcloud_ptr);
    descriptors_extractor.compute(*new_shot_ptr);
    std::cout << "new_shot_ptr: " << new_shot_ptr->points.size() << "\n";

    // Initialise the KdTreeFLANN search for the old keypoint pointcloud
    pcl::KdTreeFLANN<pcl::PointXYZRGB> old_kd_tree_flann_keypoints;
    old_kd_tree_flann_keypoints.setInputCloud(old_keypoints_ptr);

    // Initialise the KdTreeFLANN search for the new keypoint pointcloud
    pcl::KdTreeFLANN<pcl::PointXYZRGB> new_kd_tree_flann_keypoints;
    new_kd_tree_flann_keypoints.setInputCloud(new_keypoints_ptr);

    // Estimate the threshold
    double pos_radius = Configurations::get_instance()->pos_radius;
    std::vector<float> descriptor_distances;
    for (size_t i = 0; i < old_keypoints_ptr->points.size(); ++i) {
        pcl::PointXYZRGB old_keypoint = old_keypoints_ptr->points[i];
        std::vector<int> possible_refer_indices;
        std::vector<float> possible_squared_distances;
        if (!new_kd_tree_flann_keypoints.radiusSearch(old_keypoint, pos_radius, possible_refer_indices, possible_squared_distances) > 0) {
            continue;
        }
        int new_descriptor_index = -1;
        float min_descriptor_d = FLT_MAX;
        for (size_t j = 0; j < possible_refer_indices.size(); ++j) {
            int new_descriptor_index_ = possible_refer_indices[j];
            float descriptor_d = 0;
            for (int k = 0; k < 1344; ++k) {
                descriptor_d += (old_shot_ptr->points[i].descriptor[k] - new_shot_ptr->points[new_descriptor_index_].descriptor[k]) *
                                (old_shot_ptr->points[i].descriptor[k] - new_shot_ptr->points[new_descriptor_index_].descriptor[k]);
            }
            descriptor_d = sqrt(descriptor_d);
            if (descriptor_d < min_descriptor_d) {
                min_descriptor_d = descriptor_d;
                new_descriptor_index = new_descriptor_index_;
            }
        }
        pcl::PointXYZRGB old_part = old_keypoint;
        pcl::PointXYZRGB new_part = new_keypoints_ptr->points[new_descriptor_index];
        if (new_descriptor_index != -1) {
            descriptor_distances.push_back(min_descriptor_d);
        }
    }
    float descriptor_distance_avg = 0;
    for (size_t i = 0; i < descriptor_distances.size(); ++i) {
        descriptor_distance_avg += descriptor_distances[i];
    }
    descriptor_distance_avg /= descriptor_distances.size();
    float descriptor_distance_threshold = descriptor_distance_avg;
    std::cout << "descriptor_distance_threshold: " << descriptor_distance_threshold << "\n";

    // Find the descriptor matches
    for (size_t i = 0; i < old_keypoints_ptr->points.size(); ++i) {
        pcl::PointXYZRGB old_keypoint = old_keypoints_ptr->points[i];
        std::vector<int> possible_refer_indices;
        std::vector<float> possible_squared_distances;
        if (!new_kd_tree_flann_keypoints.radiusSearch(old_keypoint, pos_radius, possible_refer_indices, possible_squared_distances) > 0) {
            continue;
        }
        int new_descriptor_index = -1;
        float min_descriptor_d = FLT_MAX;
        for (size_t j = 0; j < possible_refer_indices.size(); ++j) {
            int new_descriptor_index_ = possible_refer_indices[j];
            float descriptor_d = 0;
            for (int k = 0; k < 1344; ++k) {
                descriptor_d += (old_shot_ptr->points[i].descriptor[k] - new_shot_ptr->points[new_descriptor_index_].descriptor[k]) *
                                (old_shot_ptr->points[i].descriptor[k] - new_shot_ptr->points[new_descriptor_index_].descriptor[k]);
            }
            descriptor_d = sqrt(descriptor_d);
            if (descriptor_d < min_descriptor_d) {
                min_descriptor_d = descriptor_d;
                new_descriptor_index = new_descriptor_index_;
            }
        }
        if (min_descriptor_d < descriptor_distance_threshold) {
            pcl::PointXYZRGB new_keypoint = new_keypoints_ptr->points[new_descriptor_index];
            if (!old_kd_tree_flann_keypoints.radiusSearch(new_keypoint, pos_radius, possible_refer_indices, possible_squared_distances) > 0) {
                continue;
            }
            int old_descriptor_index = -1;
            float min_descriptor_d_old = FLT_MAX;
            for (size_t j = 0; j < possible_refer_indices.size(); ++j) {
                int old_descriptor_index_ = possible_refer_indices[j];
                float descriptor_d = 0;
                for (int k = 0; k < 1344; ++k) {
                    descriptor_d += (old_shot_ptr->points[old_descriptor_index_].descriptor[k] - new_shot_ptr->points[new_descriptor_index].descriptor[k]) *
                                    (old_shot_ptr->points[old_descriptor_index_].descriptor[k] - new_shot_ptr->points[new_descriptor_index].descriptor[k]);
                }
                descriptor_d = sqrt(descriptor_d);
                if (descriptor_d < min_descriptor_d_old) {
                    min_descriptor_d_old = descriptor_d;
                    old_descriptor_index = old_descriptor_index_;
                }
            }
            if (old_descriptor_index == i) {
                pcl::PointXYZRGB old_part = old_keypoint;
                pcl::PointXYZRGB new_part = new_keypoints_ptr->points[new_descriptor_index];
                old_parts_ptr->points.push_back(old_part);
                new_parts_ptr->points.push_back(new_part);
            }
        }
    }
}

void print_matching_option(const char* parameter_ptr, const int& option) {
    if (parameter_ptr == NULL || option >= sizeof(parameter_ptr) / sizeof(parameter_ptr[0])) {
        std::cout << ERROR << "\n";
        std::cout << HELP << "\n";
        return;
    }

    std::cout << "not exists " << parameter_ptr << " for " << options[option] << "\n";
}

int get_option(const char* parameter_ptr) {
    if (parameter_ptr == NULL) {
        std::cout << ERROR << "\n";
        std::cout << HELP << "\n";
        return INVALID;
    }

    std::string parameter(parameter_ptr);
    int option_num = sizeof(options) / sizeof(options[0]);

    // Loop and compare
    for (int i = 0; i < option_num; ++i) {
        if (parameter.compare(options[i]) == 0) {
            return i;
        }
    }

    std::cout << "not exists " << parameter << " options"
              << "\n";
    return INVALID;
}

int config_value_by_option(const int& option, char* parameter_ptr) {
    std::string parameter(parameter_ptr);

    if (parameter_ptr == NULL || parameter.compare("") == 0 || option >= sizeof(options) / sizeof(options[0])) {
        std::cout << ERROR << "\n";
        std::cout << HELP << "\n";
        return INVALID;
    }

    // Loop and compare
    int i = 0;
    if (option < 2) {
        int method_num = sizeof(method_name) / sizeof(method_name[0]);
        for (; i < method_num; ++i) {
            if (parameter.compare(method_name[i]) == 0) {
                break;
            }
        }
    }

    // There are 3 noise filtering methods
    if (option == 0) {
        if (i > 2)
            goto error;

        if (i == 0) {
            command_option.keypoint_detect_method.f3 = &iss_detect_keypoints;
        } else if (i == 1) {
            command_option.keypoint_detect_method.f3 = &susan_detect_keypoints;
        } else if (i == 2) {
            command_option.keypoint_detect_method.f3 = &harris3d_detect_keypoints;
        }

    }

    // There are 2 down sampling methods
    else if (option == 1) {
        if (i <= 2)
            goto error;
        if (i == 3) {
            command_option.descriptor_detect_methos.f6 = &two_dimension_detect_keypoints;
        } else if (i == 4) {
            command_option.descriptor_detect_methos.f6 = &icp_extract_description;
        } else if (i == 5) {
            command_option.descriptor_detect_methos.f6 = &shot_extract_description;
        } else if (i == 6) {
            command_option.descriptor_detect_methos.f6 = &fpfh_extract_description;
        } else if (i == 7) {
            command_option.descriptor_detect_methos.f6 = &shotcolor_extract_description;
        }
    } else if (option == 2) {
        // Yes or No
        if (parameter.compare("Y") == 0 || parameter.compare("y") == 0 ||
            parameter.compare("Yes") == 0 || parameter.compare("yes") == 0) {
            command_option.inter = true;
        } else if (parameter.compare("N") == 0 || parameter.compare("n") == 0 ||
                   parameter.compare("No") == 0 || parameter.compare("no") == 0) {
            command_option.inter = false;
        } else
            goto error;
    } else if (option == 3) {
        command_option.input1 = parameter_ptr;
    } else if (option == 4) {
        command_option.input2 = parameter_ptr;
    } else if (option == 5) {
        command_option.output = parameter_ptr;
    } else if (option == 6) {
        command_option.inter_keypoint_1 = parameter_ptr;
    } else if (option == 7) {
        command_option.inter_keypoint_2 = parameter_ptr;
    } else if (option == 8) {
        command_option.matching_pairs = parameter_ptr;
    } else if (option == 9) {
        command_option.offset1 = parameter_ptr;
    } else if (option == 10) {
        command_option.offset2 = parameter_ptr;
    }
    return 1;
error:
    print_matching_option(parameter_ptr, option);
    return INVALID;
}

int main(int argc, char* argv[]) {
    int option_index = -1;

    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    Configurations::get_instance()->read_config();

    // Loop through the option parameters
    for (int i = 1; i < argc; ++i) {
        if (i % 2 == 1) {
            option_index = get_option(argv[i]);

            // Crash program when parameters do not exist
            if (option_index == -1) {
                std::cout << ERROR << "\n";
                std::cout << HELP << "\n";
                return INVALID;
            }
        } else {
            if (option_index == -1 || config_value_by_option(option_index, argv[i]) == -1) {
                std::cout << ERROR << "\n";
                std::cout << HELP << "\n";
                return INVALID;
            }

            // Reset
            option_index = -1;
        }
    }

    // Load the old pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr old_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

    if (pcl::io::loadPLYFile(command_option.input1, *old_pointcloud_ptr)) {
        printf("Failed to load the old pointcloud!\n");
        exit(-1);
    }
    std::cout << "old_pointcloud_ptr: " << *old_pointcloud_ptr << "\n";

    // Load the new pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    if (pcl::io::loadPLYFile(command_option.input2, *new_pointcloud_ptr)) {
        printf("Failed to load the new pointcloud!\n");
        exit(-1);
    }
    std::cout << "new_pointcloud_ptr: " << *new_pointcloud_ptr << "\n";

    // Load the offset values
    double offset1_x, offset1_y, offset1_z;
    double offset2_x, offset2_y, offset2_z;
    double offset_x, offset_y, offset_z;
    std::ifstream ifs_ofs1(command_option.offset1);
    ifs_ofs1 >> std::fixed >> offset1_x;
    ifs_ofs1 >> std::fixed >> offset1_y;
    ifs_ofs1 >> std::fixed >> offset1_z;
    ifs_ofs1.close();
    std::ifstream ifs_ofs2(command_option.offset2);
    ifs_ofs2 >> std::fixed >> offset2_x;
    ifs_ofs2 >> std::fixed >> offset2_y;
    ifs_ofs2 >> std::fixed >> offset2_z;
    ifs_ofs2.close();
    offset_x = offset2_x - offset1_x;
    offset_y = offset2_y - offset1_y;
    offset_z = offset2_z - offset1_z;

    // Shift the new pointcloud by the offset values
    for (size_t i = 0; i < new_pointcloud_ptr->points.size(); ++i) {
        new_pointcloud_ptr->points[i].x += offset_x;
        new_pointcloud_ptr->points[i].y += offset_y;
        new_pointcloud_ptr->points[i].z += offset_z;
    }

    // Detect the keypoints
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr old_keypoints_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_keypoints_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    command_option.keypoint_detect_method.f3(old_pointcloud_ptr, old_keypoints_ptr, true);
    std::cout << "old_keypoints_ptr size: " << old_keypoints_ptr->points.size() << "\n";
    command_option.keypoint_detect_method.f3(new_pointcloud_ptr, new_keypoints_ptr, false);
    std::cout << "new_keypoints_ptr size: " << new_keypoints_ptr->points.size() << "\n";
    if (command_option.inter) {
        draw_keypoints(command_option.inter_keypoint_1, old_pointcloud_ptr, old_keypoints_ptr);
        draw_keypoints(command_option.inter_keypoint_2, new_pointcloud_ptr, new_keypoints_ptr);
    }

    // Extract the descriptors and match
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr old_parts_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_parts_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    command_option.descriptor_detect_methos.f6(old_pointcloud_ptr, new_pointcloud_ptr, old_keypoints_ptr, new_keypoints_ptr, old_parts_ptr, new_parts_ptr);
    std::cout << "old_parts_ptr: " << old_parts_ptr->points.size() << "\n";
    std::cout << "new_parts_ptr: " << new_parts_ptr->points.size() << "\n";
    std::ofstream ofs_pairs(command_option.matching_pairs);
    ofs_pairs << std::fixed << offset1_x << " " << std::fixed << offset1_y << " " << std::fixed << offset1_z << "\n";
    for (size_t i = 0; i < old_parts_ptr->points.size(); ++i) {
        ofs_pairs << old_parts_ptr->points[i].x << " " << old_parts_ptr->points[i].y << " " << old_parts_ptr->points[i].z << " "
                  << new_parts_ptr->points[i].x << " " << new_parts_ptr->points[i].y << " " << new_parts_ptr->points[i].z << "\n";
    }
    ofs_pairs.close();

    // Draw the matches;
    draw_matching_results(command_option.output, old_pointcloud_ptr, new_pointcloud_ptr, old_parts_ptr, new_parts_ptr);
}
