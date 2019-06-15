#include "pointcloud_matching/matching.hpp"

void drawKeypoints(std::string fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_kps) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr drawKP(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (size_t i = 0; i < p_kps->points.size(); ++i) {
        float step = Configurations::getInstance()->leaf_size / 100;
        pcl::PointXYZRGB tmp;
        tmp.r = 255;
        tmp.g = 0;
        tmp.b = 0;
        tmp.x = p_kps->points[i].x + step;
        tmp.y = p_kps->points[i].y;
        tmp.z = p_kps->points[i].z;
        drawKP->points.push_back(tmp);
        tmp.x = p_kps->points[i].x - step;
        tmp.y = p_kps->points[i].y;
        tmp.z = p_kps->points[i].z;
        drawKP->points.push_back(tmp);
        tmp.x = p_kps->points[i].x;
        tmp.y = p_kps->points[i].y + step;
        tmp.z = p_kps->points[i].z;
        drawKP->points.push_back(tmp);
        tmp.x = p_kps->points[i].x;
        tmp.y = p_kps->points[i].y - step;
        tmp.z = p_kps->points[i].z;
        drawKP->points.push_back(tmp);
        tmp.x = p_kps->points[i].x;
        tmp.y = p_kps->points[i].y;
        tmp.z = p_kps->points[i].z + step;
        drawKP->points.push_back(tmp);
        tmp.x = p_kps->points[i].x;
        tmp.y = p_kps->points[i].y;
        tmp.z = p_kps->points[i].z - step;
        drawKP->points.push_back(tmp);
    }
    for (size_t i = 0; i < p_pcl->points.size(); ++i) {
        pcl::PointXYZRGB tmp = p_pcl->points[i];
        drawKP->points.push_back(tmp);
    }
    drawKP->width = drawKP->points.size();
    drawKP->height = 1;
    pcl::io::savePLYFile(fileName, *drawKP, true);
    std::cout << fileName << " saved.\n";
}

void drawMatchingResults(std::string fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_pcl, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_pcl, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_parts, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_parts) {
    double leaf = Configurations::getInstance()->leaf_size * 2;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_matches(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (size_t i = 0; i < p_old_pcl->points.size(); ++i) {
        pcl::PointXYZRGB tmp;
        if (Configurations::getInstance()->draw_old_colour) {
            tmp.r = p_old_pcl->points[i].r;
            tmp.g = p_old_pcl->points[i].g;
            tmp.b = p_old_pcl->points[i].b;
        } else {
            tmp.r = 0;
            tmp.g = 0;
            tmp.b = 255;
        }
        tmp.x = p_old_pcl->points[i].x;
        tmp.y = p_old_pcl->points[i].y;
        tmp.z = p_old_pcl->points[i].z;
        p_matches->points.push_back(tmp);
    }
    for (size_t i = 0; i < p_new_pcl->points.size(); ++i) {
        pcl::PointXYZRGB tmp;
        if (Configurations::getInstance()->draw_new_colour) {
            tmp.r = p_new_pcl->points[i].r;
            tmp.g = p_new_pcl->points[i].g;
            tmp.b = p_new_pcl->points[i].b;
        } else {
            tmp.r = 255;
            tmp.g = 0;
            tmp.b = 0;
        }
        tmp.x = p_new_pcl->points[i].x;
        tmp.y = p_new_pcl->points[i].y;
        tmp.z = p_new_pcl->points[i].z;
        p_matches->points.push_back(tmp);
    }
    for (size_t i = 0; i < p_old_parts->points.size(); ++i) {
        pcl::PointXYZRGB vec;
        vec.x = p_new_parts->points[i].x - p_old_parts->points[i].x;
        vec.y = p_new_parts->points[i].y - p_old_parts->points[i].y;
        vec.z = p_new_parts->points[i].z - p_old_parts->points[i].z;
        float length = sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
        if (length > 0) {
            vec.x /= length;
            vec.y /= length;
            vec.z /= length;
            for (float t = 0; t < 1e10; t += leaf / 100) {
                pcl::PointXYZRGB tmp;
                tmp.r = 255 * (t / length);
                tmp.g = 255;
                tmp.b = 255 - 255 * (t / length);
                if (t > length) {
                    break;
                }
                tmp.x = p_old_parts->points[i].x + t * vec.x;
                tmp.y = p_old_parts->points[i].y + t * vec.y;
                tmp.z = p_old_parts->points[i].z + t * vec.z;
                p_matches->points.push_back(tmp);
            }
        }
    }
    p_matches->width = p_matches->points.size();
    p_matches->height = 1;
    p_matches->is_dense = 1;
    pcl::io::savePLYFile(fileName, *p_matches, true);
    std::cout << fileName << " saved.\n";
}

/* keypoints detect method */
void issDetectKeypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_kps, bool isBefore) {
    std::cout << "ISS keypoint detection.\n";
    double iss_salient_radius;
    double iss_nonmax_radius;
    int iss_min_neighbours;
    if (isBefore) {
        iss_salient_radius = Configurations::getInstance()->iss_old_salient_radius;
        iss_nonmax_radius = Configurations::getInstance()->iss_old_nonmax_radius;
        iss_min_neighbours = Configurations::getInstance()->iss_old_min_neighbours;
    } else {
        iss_salient_radius = Configurations::getInstance()->iss_new_salient_radius;
        iss_nonmax_radius = Configurations::getInstance()->iss_new_nonmax_radius;
        iss_min_neighbours = Configurations::getInstance()->iss_new_min_neighbours;
    }

    pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss_detector;
    iss_detector.setSalientRadius(iss_salient_radius);
    iss_detector.setNonMaxRadius(iss_nonmax_radius);
    iss_detector.setMinNeighbors(iss_min_neighbours);
    iss_detector.setInputCloud(p_pcl);
    iss_detector.compute(*p_kps);
}

void susanDetectKeypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_kps, bool isBefore) {
    std::cout << "Susan keypoint detection.\n";
    double susan_radius;
    if (isBefore) {
        susan_radius = Configurations::getInstance()->susan_old_radius;
    } else {
        susan_radius = Configurations::getInstance()->susan_new_radius;
    }

    pcl::SUSANKeypoint<pcl::PointXYZRGB, pcl::PointXYZRGB> susan_detector;
    susan_detector.setRadius(susan_radius);
    susan_detector.setInputCloud(p_pcl);
    susan_detector.compute(*p_kps);
}

void harris3dDetectkeypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_kps, bool isBefore) {
    std::cout << "Harris 3d keypoint detection.\n";
    double harris3d_radius = Configurations::getInstance()->harris3d_radius;
    pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI> harris3d;
    harris3d.setRefine(false);
    harris3d.setInputCloud(p_pcl);
    if (isBefore) {
        for (float hariss_octave = 1.0; hariss_octave <= 2.0; hariss_octave += 1.0) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr p_xyzi_kps(new pcl::PointCloud<pcl::PointXYZI>);
            harris3d.setRadius(harris3d_radius * hariss_octave);
            harris3d.compute(*p_xyzi_kps);
            std::cout << "p_xyzi_kps: " << p_xyzi_kps->points.size() << "\n";
            for (size_t i = 0; i < p_xyzi_kps->points.size(); ++i) {
                pcl::PointXYZI pointi = p_xyzi_kps->points[i];
                pcl::PointXYZRGB pointrgb;
                pointrgb.x = pointi.x;
                pointrgb.y = pointi.y;
                pointrgb.z = pointi.z;
                p_kps->points.push_back(pointrgb);
            }
        }
    } else {
        for (float hariss_octave = 1.0; hariss_octave <= 2.0; hariss_octave += 1.0) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr p_xyzi_kps(new pcl::PointCloud<pcl::PointXYZI>);
            harris3d.setRadius(harris3d_radius * hariss_octave);
            harris3d.compute(*p_xyzi_kps);
            std::cout << "p_xyzi_kps: " << p_xyzi_kps->points.size() << "\n";
            for (size_t i = 0; i < p_xyzi_kps->points.size(); ++i) {
                pcl::PointXYZI pointi = p_xyzi_kps->points[i];
                pcl::PointXYZRGB pointrgb;
                pointrgb.x = pointi.x;
                pointrgb.y = pointi.y;
                pointrgb.z = pointi.z;
                p_kps->points.push_back(pointrgb);
            }
        }
    }
    p_kps->width = p_kps->points.size();
    p_kps->height = 1;
}

void twoDimensionDetectKeypoints(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_pcl,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_pcl,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_kps,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_kps,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_parts,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_parts) {
    std::cout << "2D matching method.\n";
    CloudProjection clp_instance(p_old_pcl, p_new_pcl, p_old_parts, p_new_parts);
    clp_instance.detect_matches();
}

/* descriptor detect method */
void icpDetectDescriptor(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_pcl,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_pcl,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_kps,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_kps,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_parts,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_parts) {
    std::cout << "ICP matching.\n";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_pcl_icp(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_pcl_icp(new pcl::PointCloud<pcl::PointXYZRGB>());

    pcl::VoxelGrid<pcl::PointXYZRGB> grid;
    double leaf = Configurations::getInstance()->leaf_size * 2.5;
    grid.setLeafSize(leaf, leaf, leaf);
    grid.setInputCloud(p_old_pcl);
    grid.filter(*p_old_pcl_icp);
    std::cout << "p_old_pcl_icp: " << *p_old_pcl_icp << "\n";
    grid.setInputCloud(p_new_pcl);
    grid.filter(*p_new_pcl_icp);
    std::cout << "p_new_pcl_icp: " << *p_new_pcl_icp << "\n";

    double icp_radius = Configurations::getInstance()->icp_radius;
    int icp_iterations = Configurations::getInstance()->icp_iterations;
    double pos_radius = Configurations::getInstance()->pos_radius;
    double icp_refine_radius = Configurations::getInstance()->icp_refine_radius;
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_old_pcl;
    kd_old_pcl.setInputCloud(p_old_pcl_icp);
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_new_pcl;
    kd_new_pcl.setInputCloud(p_new_pcl_icp);
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_new_kps;
    kd_new_kps.setInputCloud(p_new_kps);

    // Search most similar point from possile regions
    for (size_t i = 0; i < p_old_kps->points.size(); ++i) {
        std::cout << " Matching process: " << i << "/" << p_old_kps->points.size() << "\n";
        pcl::PointXYZRGB old_kpt = (p_old_kps->points)[i];
        std::vector<int> old_neighbour_index;
        std::vector<float> old_neighbours_sqd;
        if (!kd_old_pcl.radiusSearch(old_kpt, icp_radius, old_neighbour_index, old_neighbours_sqd) > 0) {
            std::cout << " !not found old neighbours\n";
            continue;
        }
        // Old keypoint's neighbours
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr old_neighbours(new pcl::PointCloud<pcl::PointXYZRGB>());
        for (size_t j = 0; j < old_neighbour_index.size(); ++j) {
            old_neighbours->points.push_back(p_old_pcl_icp->points[old_neighbour_index[j]]);
        }

        // Get possible refer keypoints
        std::vector<int> pos_refer_index;
        std::vector<float> pos_refer_sqd;
        if (!kd_new_kps.radiusSearch(old_kpt, pos_radius, pos_refer_index, pos_refer_sqd) > 0) {
            std::cout << " !not found possible refer keypoint.\n";
            continue;
        }
        float best_score = 1e10;
        int best_refer_index = 0;
        for (size_t j = 0; j < pos_refer_index.size(); ++j) {
            pcl::PointXYZRGB new_kpt = (p_new_kps->points)[pos_refer_index[j]];
            std::vector<int> new_neighbour_index;
            std::vector<float> new_neighbours_sqd;
            if (!kd_new_pcl.radiusSearch(new_kpt, icp_radius, new_neighbour_index, new_neighbours_sqd) > 0) {
                std::cout << " !not found new neighbours\n";
                continue;
            }
            // New keypoint's neighbours
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_neighbours(new pcl::PointCloud<pcl::PointXYZRGB>());
            for (size_t k = 0; k < new_neighbour_index.size(); ++k) {
                new_neighbours->points.push_back(p_new_pcl_icp->points[new_neighbour_index[k]]);
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
                    std::cout << " best index: " << best_refer_index << " score: " << best_score << " refer point " << j << "/" << pos_refer_index.size() << " size icp " << new_neighbours->points.size() << " -> " << old_neighbours->points.size() << "\n";
                }
            }
        }
        if (best_score < 1e3) {
            std::cout << "Refine";
            // Get refine points
            pcl::PointXYZRGB similar_kpt = p_new_kps->points[pos_refer_index[best_refer_index]];
            std::vector<int> refine_index;
            std::vector<float> refine_sqd;
            if (!kd_new_pcl.radiusSearch(similar_kpt, icp_refine_radius, refine_index, refine_sqd) > 0) {
                std::cout << " !not found refine point\n";
                continue;
            }
            float refine_best_score = 1e10;
            int refine_best_refer_index = 0;
            float diff_size_ratio = 1;
            for (size_t j = 0; j < refine_index.size(); ++j) {
                pcl::PointXYZRGB new_similar_point = (p_new_pcl_icp->points)[refine_index[j]];
                std::vector<int> new_neighbour_index;
                std::vector<float> new_neighbours_sqd;
                if (!kd_new_pcl.radiusSearch(new_similar_point, icp_radius, new_neighbour_index, new_neighbours_sqd) > 0) {
                    std::cout << " !not found new neighbours\n";
                    continue;
                }
                // New keypoint's neighbours
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_neighbours(new pcl::PointCloud<pcl::PointXYZRGB>());
                for (size_t k = 0; k < new_neighbour_index.size(); ++k) {
                    new_neighbours->points.push_back(p_new_pcl_icp->points[new_neighbour_index[k]]);
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
            pcl::PointXYZRGB nearestPoint = p_new_pcl_icp->points[refine_index[refine_best_refer_index]];
            pcl::PointXYZRGB old_part = old_kpt;
            old_part.r = 255 * diff_size_ratio;
            old_part.g = 255 * diff_size_ratio;
            old_part.b = 255 * diff_size_ratio;
            p_old_parts->points.push_back(old_part);
            p_new_parts->points.push_back(nearestPoint);
        }
    }
}

void shotDetectDescriptor(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_pcl,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_pcl,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_kps,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_kps,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_parts,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_parts) {
    std::cout << "SHOT matching.\n";
    // Normal estimation
    pcl::PointCloud<pcl::Normal>::Ptr p_old_normal(new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::Normal>::Ptr p_new_normal(new pcl::PointCloud<pcl::Normal>());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(10);
    norm_est.setInputCloud(p_old_pcl);
    norm_est.compute(*p_old_normal);
    std::cout << "p_old_normal: " << p_old_normal->points.size() << "\n";
    norm_est.setInputCloud(p_new_pcl);
    norm_est.compute(*p_new_normal);
    std::cout << "p_new_normal: " << p_new_normal->points.size() << "\n";

    // SHOT extraction
    double shot_radius = Configurations::getInstance()->shot_radius;
    pcl::SHOTEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352> descr_est;
    descr_est.setRadiusSearch(shot_radius);
    descr_est.setInputCloud(p_old_kps);
    descr_est.setInputNormals(p_old_normal);
    descr_est.setSearchSurface(p_old_pcl);
    pcl::PointCloud<pcl::SHOT352>::Ptr p_old_shot(new pcl::PointCloud<pcl::SHOT352>());
    descr_est.compute(*p_old_shot);
    std::cout << "p_old_shot: " << p_old_shot->points.size() << "\n";

    descr_est.setInputCloud(p_new_kps);
    descr_est.setInputNormals(p_new_normal);
    descr_est.setSearchSurface(p_new_pcl);
    pcl::PointCloud<pcl::SHOT352>::Ptr p_new_shot(new pcl::PointCloud<pcl::SHOT352>());
    descr_est.compute(*p_new_shot);
    std::cout << "p_new_shot: " << p_new_shot->points.size() << "\n";

    // Threshold estimation
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_old_kps;
    kd_old_kps.setInputCloud(p_old_kps);
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_new_kps;
    kd_new_kps.setInputCloud(p_new_kps);
    double pos_radius = Configurations::getInstance()->pos_radius;
    std::vector<float> des_distances;
    for (size_t i = 0; i < p_old_kps->points.size(); ++i) {
        pcl::PointXYZRGB old_kpt = p_old_kps->points[i];

        std::vector<int> pos_refer_index;
        std::vector<float> pos_refer_sqd;
        if (!kd_new_kps.radiusSearch(old_kpt, pos_radius, pos_refer_index, pos_refer_sqd) > 0) {
            continue;
        }
        int new_des_idx = -1;
        float min_des_d = 1e10;
        for (size_t j = 0; j < pos_refer_index.size(); ++j) {
            int new_des_idx_ = pos_refer_index[j];
            float des_d = 0;
            for (int k = 0; k < 352; ++k) {
                des_d += (p_old_shot->points[i].descriptor[k] - p_new_shot->points[new_des_idx_].descriptor[k]) *
                         (p_old_shot->points[i].descriptor[k] - p_new_shot->points[new_des_idx_].descriptor[k]);
            }
            des_d = sqrt(des_d);
            if (des_d < min_des_d) {
                min_des_d = des_d;
                new_des_idx = new_des_idx_;
            }
        }
        pcl::PointXYZRGB old_part = old_kpt;
        pcl::PointXYZRGB new_part = p_new_kps->points[new_des_idx];
        if (new_des_idx != -1) {
            des_distances.push_back(min_des_d);
        }
    }
    float des_distance_avg = 0;
    for (size_t i = 0; i < des_distances.size(); ++i) {
        des_distance_avg += des_distances[i];
    }
    des_distance_avg /= des_distances.size();
    float des_distance_threshold = des_distance_avg;
    std::cout << "des_distance_threshold: " << des_distance_threshold << "\n";

    // Matching
    for (size_t i = 0; i < p_old_kps->points.size(); ++i) {
        pcl::PointXYZRGB old_kpt = p_old_kps->points[i];

        std::vector<int> pos_refer_index;
        std::vector<float> pos_refer_sqd;
        if (!kd_new_kps.radiusSearch(old_kpt, pos_radius, pos_refer_index, pos_refer_sqd) > 0) {
            continue;
        }
        int new_des_idx = -1;
        float min_des_d = 1e10;
        for (size_t j = 0; j < pos_refer_index.size(); ++j) {
            int new_des_idx_ = pos_refer_index[j];
            float des_d = 0;
            for (int k = 0; k < 352; ++k) {
                des_d += (p_old_shot->points[i].descriptor[k] - p_new_shot->points[new_des_idx_].descriptor[k]) *
                         (p_old_shot->points[i].descriptor[k] - p_new_shot->points[new_des_idx_].descriptor[k]);
            }
            des_d = sqrt(des_d);
            if (des_d < min_des_d) {
                min_des_d = des_d;
                new_des_idx = new_des_idx_;
            }
        }
        if (min_des_d < des_distance_threshold) {
            pcl::PointXYZRGB new_kpt = p_new_kps->points[new_des_idx];
            if (!kd_old_kps.radiusSearch(new_kpt, pos_radius, pos_refer_index, pos_refer_sqd) > 0) {
                continue;
            }
            int old_des_idx = -1;
            float min_des_d_old = 1e10;
            for (size_t j = 0; j < pos_refer_index.size(); ++j) {
                int old_des_idx_ = pos_refer_index[j];
                float des_d = 0;
                for (int k = 0; k < 352; ++k) {
                    des_d += (p_old_shot->points[old_des_idx_].descriptor[k] - p_new_shot->points[new_des_idx].descriptor[k]) *
                             (p_old_shot->points[old_des_idx_].descriptor[k] - p_new_shot->points[new_des_idx].descriptor[k]);
                }
                des_d = sqrt(des_d);
                if (des_d < min_des_d_old) {
                    min_des_d_old = des_d;
                    old_des_idx = old_des_idx_;
                }
            }
            if (old_des_idx == i) {
                pcl::PointXYZRGB old_part = old_kpt;
                pcl::PointXYZRGB new_part = p_new_kps->points[new_des_idx];
                p_old_parts->points.push_back(old_part);
                p_new_parts->points.push_back(new_part);
            }
        }
    }
}

void shotcolorDetectDescriptor(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_pcl,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_pcl,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_kps,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_kps,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_parts,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_parts) {
    std::cout << "SHOTCOLOR matching.\n";
    normalizeColours(p_old_pcl);
    normalizeColours(p_new_pcl);
    // Normal estimation
    pcl::PointCloud<pcl::Normal>::Ptr p_old_normal(new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::Normal>::Ptr p_new_normal(new pcl::PointCloud<pcl::Normal>());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(10);
    norm_est.setInputCloud(p_old_pcl);
    norm_est.compute(*p_old_normal);
    std::cout << "p_old_normal: " << p_old_normal->points.size() << "\n";
    norm_est.setInputCloud(p_new_pcl);
    norm_est.compute(*p_new_normal);
    std::cout << "p_new_normal: " << p_new_normal->points.size() << "\n";

    // SHOTCOLOR extraction
    double shot_radius = Configurations::getInstance()->shot_radius;
    pcl::SHOTColorEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344> descr_est;
    descr_est.setRadiusSearch(shot_radius);
    descr_est.setInputCloud(p_old_kps);
    descr_est.setInputNormals(p_old_normal);
    descr_est.setSearchSurface(p_old_pcl);
    pcl::PointCloud<pcl::SHOT1344>::Ptr p_old_shot(new pcl::PointCloud<pcl::SHOT1344>());
    descr_est.compute(*p_old_shot);
    std::cout << "p_old_shot: " << p_old_shot->points.size() << "\n";

    descr_est.setInputCloud(p_new_kps);
    descr_est.setInputNormals(p_new_normal);
    descr_est.setSearchSurface(p_new_pcl);
    pcl::PointCloud<pcl::SHOT1344>::Ptr p_new_shot(new pcl::PointCloud<pcl::SHOT1344>());
    descr_est.compute(*p_new_shot);
    std::cout << "p_new_shot: " << p_new_shot->points.size() << "\n";

    // Threshold estimation
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_old_kps;
    kd_old_kps.setInputCloud(p_old_kps);
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_new_kps;
    kd_new_kps.setInputCloud(p_new_kps);
    double pos_radius = Configurations::getInstance()->pos_radius;
    std::vector<float> des_distances;
    for (size_t i = 0; i < p_old_kps->points.size(); ++i) {
        pcl::PointXYZRGB old_kpt = p_old_kps->points[i];

        std::vector<int> pos_refer_index;
        std::vector<float> pos_refer_sqd;
        if (!kd_new_kps.radiusSearch(old_kpt, pos_radius, pos_refer_index, pos_refer_sqd) > 0) {
            continue;
        }
        int new_des_idx = -1;
        float min_des_d = 1e10;
        for (size_t j = 0; j < pos_refer_index.size(); ++j) {
            int new_des_idx_ = pos_refer_index[j];
            float des_d = 0;
            for (int k = 0; k < 1344; ++k) {
                des_d += (p_old_shot->points[i].descriptor[k] - p_new_shot->points[new_des_idx_].descriptor[k]) *
                         (p_old_shot->points[i].descriptor[k] - p_new_shot->points[new_des_idx_].descriptor[k]);
            }
            des_d = sqrt(des_d);
            if (des_d < min_des_d) {
                min_des_d = des_d;
                new_des_idx = new_des_idx_;
            }
        }
        pcl::PointXYZRGB old_part = old_kpt;
        pcl::PointXYZRGB new_part = p_new_kps->points[new_des_idx];
        if (new_des_idx != -1) {
            des_distances.push_back(min_des_d);
        }
    }
    float des_distance_avg = 0;
    for (size_t i = 0; i < des_distances.size(); ++i) {
        des_distance_avg += des_distances[i];
    }
    des_distance_avg /= des_distances.size();
    float des_distance_threshold = des_distance_avg;
    std::cout << "des_distance_threshold: " << des_distance_threshold << "\n";

    //Matching
    for (size_t i = 0; i < p_old_kps->points.size(); ++i) {
        pcl::PointXYZRGB old_kpt = p_old_kps->points[i];

        std::vector<int> pos_refer_index;
        std::vector<float> pos_refer_sqd;
        if (!kd_new_kps.radiusSearch(old_kpt, pos_radius, pos_refer_index, pos_refer_sqd) > 0) {
            continue;
        }
        int new_des_idx = -1;
        float min_des_d = 1e10;
        for (size_t j = 0; j < pos_refer_index.size(); ++j) {
            int new_des_idx_ = pos_refer_index[j];
            float des_d = 0;
            for (int k = 0; k < 1344; ++k) {
                des_d += (p_old_shot->points[i].descriptor[k] - p_new_shot->points[new_des_idx_].descriptor[k]) *
                         (p_old_shot->points[i].descriptor[k] - p_new_shot->points[new_des_idx_].descriptor[k]);
            }
            des_d = sqrt(des_d);
            if (des_d < min_des_d) {
                min_des_d = des_d;
                new_des_idx = new_des_idx_;
            }
        }
        if (min_des_d < des_distance_threshold) {
            pcl::PointXYZRGB new_kpt = p_new_kps->points[new_des_idx];
            if (!kd_old_kps.radiusSearch(new_kpt, pos_radius, pos_refer_index, pos_refer_sqd) > 0) {
                continue;
            }
            int old_des_idx = -1;
            float min_des_d_old = 1e10;
            for (size_t j = 0; j < pos_refer_index.size(); ++j) {
                int old_des_idx_ = pos_refer_index[j];
                float des_d = 0;
                for (int k = 0; k < 1344; ++k) {
                    des_d += (p_old_shot->points[old_des_idx_].descriptor[k] - p_new_shot->points[new_des_idx].descriptor[k]) *
                             (p_old_shot->points[old_des_idx_].descriptor[k] - p_new_shot->points[new_des_idx].descriptor[k]);
                }
                des_d = sqrt(des_d);
                if (des_d < min_des_d_old) {
                    min_des_d_old = des_d;
                    old_des_idx = old_des_idx_;
                }
            }
            if (old_des_idx == i) {
                pcl::PointXYZRGB old_part = old_kpt;
                pcl::PointXYZRGB new_part = p_new_kps->points[new_des_idx];
                p_old_parts->points.push_back(old_part);
                p_new_parts->points.push_back(new_part);
            }
        }
    }
}

void fpfhDetectDescriptor(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_pcl,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_pcl,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_kps,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_kps,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_parts,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_parts) {
    std::cout << "FPFH matching.\n";
    // Normal estimation
    pcl::PointCloud<pcl::Normal>::Ptr p_old_normal(new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::Normal>::Ptr p_new_normal(new pcl::PointCloud<pcl::Normal>());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(10);
    norm_est.setInputCloud(p_old_pcl);
    norm_est.compute(*p_old_normal);
    std::cout << "p_old_normal: " << p_old_normal->points.size() << "\n";
    norm_est.setInputCloud(p_new_pcl);
    norm_est.compute(*p_new_normal);
    std::cout << "p_new_normal: " << p_new_normal->points.size() << "\n";

    // Take keypoints' index
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_old_pcl;
    kd_old_pcl.setInputCloud(p_old_pcl);
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_new_pcl;
    kd_new_pcl.setInputCloud(p_new_pcl);
    boost::shared_ptr<std::vector<int>> old_kpt_idx_in_pcl(new std::vector<int>(0));
    for (size_t i = 0; i < p_old_kps->points.size(); ++i) {
        pcl::PointXYZRGB old_point = p_old_kps->points[i];
        std::vector<int> old_neighbour_index;
        std::vector<float> old_neighbours_sqd;
        if (!kd_old_pcl.nearestKSearch(old_point, 1, old_neighbour_index, old_neighbours_sqd) > 0) {
            std::cout << " Check get old keypoint's indices.\n";
            exit(1);
        }
        old_kpt_idx_in_pcl->push_back(old_neighbour_index[0]);
    }
    std::cout << "old_kpt_idx_in_pcl: " << old_kpt_idx_in_pcl->size() << "\n";

    boost::shared_ptr<std::vector<int>> new_kpt_idx_in_pcl(new std::vector<int>(0));
    for (size_t i = 0; i < p_new_kps->points.size(); ++i) {
        pcl::PointXYZRGB new_point = p_new_kps->points[i];
        std::vector<int> new_neighbour_index;
        std::vector<float> new_neighbours_sqd;
        if (!kd_new_pcl.nearestKSearch(new_point, 1, new_neighbour_index, new_neighbours_sqd) > 0) {
            std::cout << " Check get new keypoint's indices.\n";
            exit(1);
        }
        new_kpt_idx_in_pcl->push_back(new_neighbour_index[0]);
    }
    std::cout << "new_kpt_idx_in_pcl: " << new_kpt_idx_in_pcl->size() << "\n";

    // FPFH estimation
    double fpfh_radius = Configurations::getInstance()->fpfh_radius;
    pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> descr_est;
    norm_est.setSearchMethod(tree);
    descr_est.setRadiusSearch(fpfh_radius);
    descr_est.setInputCloud(p_old_pcl);
    descr_est.setIndices(old_kpt_idx_in_pcl);
    descr_est.setInputNormals(p_old_normal);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr p_old_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>());
    descr_est.compute(*p_old_fpfh);
    std::cout << "p_old_fpfh: " << p_old_fpfh->points.size() << "\n";

    descr_est.setInputCloud(p_new_pcl);
    descr_est.setIndices(new_kpt_idx_in_pcl);
    descr_est.setInputNormals(p_new_normal);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr p_new_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>());
    descr_est.compute(*p_new_fpfh);
    std::cout << "p_new_fpfh: " << p_new_fpfh->points.size() << "\n";

    // Threshold estimation
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_old_kps;
    kd_old_kps.setInputCloud(p_old_kps);
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_new_kps;
    kd_new_kps.setInputCloud(p_new_kps);
    double pos_radius = Configurations::getInstance()->pos_radius;
    std::vector<float> des_distances;
    for (size_t i = 0; i < p_old_kps->points.size(); ++i) {
        pcl::PointXYZRGB old_kpt = p_old_kps->points[i];

        std::vector<int> pos_refer_index;
        std::vector<float> pos_refer_sqd;
        if (!kd_new_kps.radiusSearch(old_kpt, pos_radius, pos_refer_index, pos_refer_sqd) > 0) {
            continue;
        }
        int new_des_idx = -1;
        float min_des_d = 1e10;
        for (size_t j = 0; j < pos_refer_index.size(); ++j) {
            int new_des_idx_ = pos_refer_index[j];
            float des_d = 0;
            for (int k = 0; k < 33; ++k) {
                des_d += (p_old_fpfh->points[i].histogram[k] - p_new_fpfh->points[new_des_idx_].histogram[k]) *
                         (p_old_fpfh->points[i].histogram[k] - p_new_fpfh->points[new_des_idx_].histogram[k]);
            }
            des_d = sqrt(des_d);
            if (des_d < min_des_d) {
                min_des_d = des_d;
                new_des_idx = new_des_idx_;
            }
        }
        pcl::PointXYZRGB old_part = old_kpt;
        pcl::PointXYZRGB new_part = p_new_kps->points[new_des_idx];
        if (new_des_idx != -1) {
            des_distances.push_back(min_des_d);
        }
    }
    float des_distance_avg = 0;
    for (size_t i = 0; i < des_distances.size(); ++i) {
        des_distance_avg += des_distances[i];
    }
    des_distance_avg /= des_distances.size();
    float des_distance_threshold = des_distance_avg;
    std::cout << "des_distance_threshold: " << des_distance_threshold << "\n";

    //Matching
    for (size_t i = 0; i < p_old_kps->points.size(); ++i) {
        pcl::PointXYZRGB old_kpt = p_old_kps->points[i];

        std::vector<int> pos_refer_index;
        std::vector<float> pos_refer_sqd;
        if (!kd_new_kps.radiusSearch(old_kpt, pos_radius, pos_refer_index, pos_refer_sqd) > 0) {
            continue;
        }
        int new_des_idx = -1;
        float min_des_d = 1e10;
        for (size_t j = 0; j < pos_refer_index.size(); ++j) {
            int new_des_idx_ = pos_refer_index[j];
            float des_d = 0;
            for (int k = 0; k < 33; ++k) {
                des_d += (p_old_fpfh->points[i].histogram[k] - p_new_fpfh->points[new_des_idx_].histogram[k]) *
                         (p_old_fpfh->points[i].histogram[k] - p_new_fpfh->points[new_des_idx_].histogram[k]);
            }
            des_d = sqrt(des_d);
            if (des_d < min_des_d) {
                min_des_d = des_d;
                new_des_idx = new_des_idx_;
            }
        }
        if (min_des_d < des_distance_threshold) {
            pcl::PointXYZRGB new_kpt = p_new_kps->points[new_des_idx];
            if (!kd_old_kps.radiusSearch(new_kpt, pos_radius, pos_refer_index, pos_refer_sqd) > 0) {
                continue;
            }
            int old_des_idx = -1;
            float min_des_d_old = 1e10;
            for (size_t j = 0; j < pos_refer_index.size(); ++j) {
                int old_des_idx_ = pos_refer_index[j];
                float des_d = 0;
                for (int k = 0; k < 33; ++k) {
                    des_d += (p_old_fpfh->points[old_des_idx_].histogram[k] - p_new_fpfh->points[new_des_idx].histogram[k]) *
                             (p_old_fpfh->points[old_des_idx_].histogram[k] - p_new_fpfh->points[new_des_idx].histogram[k]);
                }
                des_d = sqrt(des_d);
                if (des_d < min_des_d_old) {
                    min_des_d_old = des_d;
                    old_des_idx = old_des_idx_;
                }
            }
            if (old_des_idx == i) {
                pcl::PointXYZRGB old_part = old_kpt;
                pcl::PointXYZRGB new_part = p_new_kps->points[new_des_idx];
                p_old_parts->points.push_back(old_part);
                p_new_parts->points.push_back(new_part);
            }
        }
    }
}

void PrintMatchingOption(char* para, int option) {
    if (para == NULL || option >= sizeof(para) / sizeof(para[0])) {
        std::cout << ERROR << "\n";
        std::cout << HELP << "\n";
        return;
    }

    std::cout << "not exists " << para << " for " << options[option] << "\n";
}

int getOption(char* _p) {
    if (_p == NULL) {
        std::cout << ERROR << "\n";
        std::cout << HELP << "\n";
        return UNVALID;
    }

    std::string para(_p);
    int optionNum = sizeof(options) / sizeof(options[0]);
    // loop and compare
    for (int i = 0; i < optionNum; i++) {
        if (para.compare(options[i]) == 0) {
            return i;
        }
    }

    std::cout << "not exists " << para << " options"
              << "\n";
    return UNVALID;
}

int configValueByOption(int option, char* _p) {
    std::string para(_p);

    if (_p == NULL || para.compare("") == 0 || option >= sizeof(options) / sizeof(options[0])) {
        std::cout << ERROR << "\n";
        std::cout << HELP << "\n";
        return UNVALID;
    }

    // loop and compare
    int i = 0;
    if (option < 2) {
        int methodNum = sizeof(methodName) / sizeof(methodName[0]);
        for (; i < methodNum; i++) {
            if (para.compare(methodName[i]) == 0) {
                break;
            }
        }
    }

    // noise, there are 3 methods
    if (option == 0) {
        if (i > 2)
            goto error;

        if (i == 0) {
            commandOption.keypoint_detect_method.f3 = &issDetectKeypoints;
        } else if (i == 1) {
            commandOption.keypoint_detect_method.f3 = &susanDetectKeypoints;
        } else if (i == 2) {
            commandOption.keypoint_detect_method.f3 = &harris3dDetectkeypoints;
        }

    }
    // down sample, there are two methods
    else if (option == 1) {
        if (i <= 2)
            goto error;
        if (i == 3) {
            commandOption.descriptor_detect_methos.f6 = &twoDimensionDetectKeypoints;
        } else if (i == 4) {
            commandOption.descriptor_detect_methos.f6 = &icpDetectDescriptor;
        } else if (i == 5) {
            commandOption.descriptor_detect_methos.f6 = &shotDetectDescriptor;
        } else if (i == 6) {
            commandOption.descriptor_detect_methos.f6 = &fpfhDetectDescriptor;
        } else if (i == 7) {
            commandOption.descriptor_detect_methos.f6 = &shotcolorDetectDescriptor;
        }
    } else if (option == 2) {
        // Yes or No
        if (para.compare("Y") == 0 || para.compare("y") == 0 ||
            para.compare("Yes") == 0 || para.compare("yes") == 0) {
            commandOption.inter = true;
        } else if (para.compare("N") == 0 || para.compare("n") == 0 ||
                   para.compare("No") == 0 || para.compare("no") == 0) {
            commandOption.inter = false;
        } else
            goto error;
    } else if (option == 3) {
        commandOption.input1 = _p;
    } else if (option == 4) {
        commandOption.input2 = _p;
    } else if (option == 5) {
        commandOption.output = _p;
    } else if (option == 6) {
        commandOption.interKeypoint1 = _p;
    } else if (option == 7) {
        commandOption.interKeypoint2 = _p;
    } else if (option == 8) {
        commandOption.matchingPairs = _p;
    } else if (option == 9) {
        commandOption.offset1 = _p;
    } else if (option == 10) {
        commandOption.offset2 = _p;
    }
    return 1;
error:
    PrintMatchingOption(_p, option);
    return UNVALID;
}

int main(int argc, char* argv[]) {
    int optionIndex = -1;

    Configurations::getInstance()->readConfig();
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    // loop thought option parameter
    for (int i = 1; i < argc; i++) {
        if (i % 2 == 1) {
            optionIndex = getOption(argv[i]);

            // crash program when param is not exists
            if (optionIndex == -1) {
                std::cout << ERROR << "\n";
                std::cout << HELP << "\n";
                return UNVALID;
            }
        } else {
            if (optionIndex == -1 || configValueByOption(optionIndex, argv[i]) == -1) {
                std::cout << ERROR << "\n";
                std::cout << HELP << "\n";
                return UNVALID;
            }

            // reset
            optionIndex = -1;
        }
    }

    // Load point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());

    if (pcl::io::loadPLYFile(commandOption.input1, *p_old_pcl)) {
        std::cout << "Error loading old pointcloud!\n";
        exit(-1);
    }
    std::cout << "p_old_pcl: " << *p_old_pcl << "\n";
    if (pcl::io::loadPLYFile(commandOption.input2, *p_new_pcl)) {
        std::cout << "Error loading new pointcloud!\n";
        exit(-1);
    }
    std::cout << "p_new_pcl: " << *p_new_pcl << "\n";

    double offset1_x, offset1_y, offset1_z;
    double offset2_x, offset2_y, offset2_z;
    double offset_x, offset_y, offset_z;
    std::ifstream ifs_ofs1(commandOption.offset1);
    ifs_ofs1 >> std::fixed >> offset1_x;
    ifs_ofs1 >> std::fixed >> offset1_y;
    ifs_ofs1 >> std::fixed >> offset1_z;
    ifs_ofs1.close();
    std::ifstream ifs_ofs2(commandOption.offset2);
    ifs_ofs2 >> std::fixed >> offset2_x;
    ifs_ofs2 >> std::fixed >> offset2_y;
    ifs_ofs2 >> std::fixed >> offset2_z;
    ifs_ofs2.close();
    offset_x = offset2_x - offset1_x;
    offset_y = offset2_y - offset1_y;
    offset_z = offset2_z - offset1_z;
    for (size_t i = 0; i < p_new_pcl->points.size(); ++i) {
        p_new_pcl->points[i].x += offset_x;
        p_new_pcl->points[i].y += offset_y;
        p_new_pcl->points[i].z += offset_z;
    }

    // Detect key points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_kps(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_kps(new pcl::PointCloud<pcl::PointXYZRGB>());
    commandOption.keypoint_detect_method.f3(p_old_pcl, p_old_kps, true);
    std::cout << "p_old_kps size: " << p_old_kps->points.size() << "\n";
    commandOption.keypoint_detect_method.f3(p_new_pcl, p_new_kps, false);
    std::cout << "p_new_kps size: " << p_new_kps->points.size() << "\n";
    if (commandOption.inter) {
        drawKeypoints(commandOption.interKeypoint1, p_old_pcl, p_old_kps);
        drawKeypoints(commandOption.interKeypoint2, p_new_pcl, p_new_kps);
    }

    // Compute descriptor and matching
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_old_parts(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_new_parts(new pcl::PointCloud<pcl::PointXYZRGB>());
    commandOption.descriptor_detect_methos.f6(p_old_pcl, p_new_pcl, p_old_kps, p_new_kps, p_old_parts, p_new_parts);
    std::cout << "p_old_parts: " << p_old_parts->points.size() << "\n";
    std::cout << "p_new_parts: " << p_new_parts->points.size() << "\n";
    std::ofstream ofs_pairs(commandOption.matchingPairs);
    ofs_pairs << std::fixed << offset1_x << " " << std::fixed << offset1_y << " " << std::fixed << offset1_z << "\n";
    for (size_t i = 0; i < p_old_parts->points.size(); ++i) {
        ofs_pairs << p_old_parts->points[i].x << " " << p_old_parts->points[i].y << " " << p_old_parts->points[i].z << " " << p_new_parts->points[i].x << " " << p_new_parts->points[i].y << " " << p_new_parts->points[i].z << "\n";
    }
    ofs_pairs.close();

    // Draw matches;
    drawMatchingResults(commandOption.output, p_old_pcl, p_new_pcl, p_old_parts, p_new_parts);
}
