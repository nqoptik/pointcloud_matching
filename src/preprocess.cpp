#include "pointcloud_matching/preprocess.hpp"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr nearest_down_sampling_with_leaf_size(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pcl_ptr,
                                                                            const double leaf_size) {
    std::cout << "Nearest median down sampling.\n";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr median_pcl_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.setInputCloud(input_pcl_ptr);
    voxel_grid.filter(*median_pcl_ptr);

    // Down sampling with real point near centre
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_tree_flann;
    kd_tree_flann.setInputCloud(input_pcl_ptr);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr nearest_pcl_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (size_t i = 0; i < median_pcl_ptr->points.size(); ++i) {
        pcl::PointXYZRGB centre_point = median_pcl_ptr->points[i];
        std::vector<int> k_indices;
        std::vector<float> k_squared_distances;
        if (kd_tree_flann.nearestKSearch(centre_point, 1, k_indices, k_squared_distances) > 0) {
            if (k_squared_distances[0] < leaf_size * leaf_size) {
                nearest_pcl_ptr->points.push_back(input_pcl_ptr->points[k_indices[0]]);
            }
        }
    }
    if (nearest_pcl_ptr->points.size() != median_pcl_ptr->points.size()) {
        std::cout << "Need to check down sampling with real point centre.\n";
        exit(1);
    }
    nearest_pcl_ptr->width = nearest_pcl_ptr->points.size();
    nearest_pcl_ptr->height = 1;
    return nearest_pcl_ptr;
}

// The noise filtering functions
pcl::PointCloud<pcl::PointXYZRGB>::Ptr stat_filtering_noise(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pcl_ptr) {
    std::cout << "Statistical outliers removal.\n";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr stat_pcl_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (int loop = 0; loop < 1; ++loop) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statistical_outlier_removal;
        statistical_outlier_removal.setInputCloud(input_pcl_ptr);
        statistical_outlier_removal.setMeanK(Configurations::get_instance()->sor_neighbours);
        statistical_outlier_removal.setStddevMulThresh(Configurations::get_instance()->sor_stdev_thresh);
        statistical_outlier_removal.filter(*stat_pcl_ptr);
        std::cout << "Loop: " << loop << " remaining cloud: " << stat_pcl_ptr->points.size() << "\n";
    }
    for (int loop = 1; loop < Configurations::get_instance()->sor_iterations; ++loop) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statistical_outlier_removal;
        statistical_outlier_removal.setInputCloud(stat_pcl_ptr);
        statistical_outlier_removal.setMeanK(Configurations::get_instance()->sor_neighbours);
        statistical_outlier_removal.setStddevMulThresh(Configurations::get_instance()->sor_stdev_thresh);
        statistical_outlier_removal.filter(*stat_pcl_ptr);
        std::cout << "Loop: " << loop << " remaining cloud: " << stat_pcl_ptr->points.size() << "\n";
    }
    return stat_pcl_ptr;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr radius_filtering_noise(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pcl_ptr) {
    std::cout << "Radius outliers removal.\n";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr radius_pcl_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (int loop = 0; loop < 1; ++loop) {
        pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> radius_outlier_removal;
        radius_outlier_removal.setInputCloud(input_pcl_ptr);
        radius_outlier_removal.setRadiusSearch(Configurations::get_instance()->ror_radius);
        radius_outlier_removal.setMinNeighborsInRadius(Configurations::get_instance()->ror_min_neighbours);
        radius_outlier_removal.filter(*radius_pcl_ptr);
        std::cout << "Loop: " << loop << " remaining cloud: " << radius_pcl_ptr->points.size() << "\n";
    }
    for (int loop = 1; loop < Configurations::get_instance()->ror_iterations; ++loop) {
        pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> radius_outlier_removal;
        radius_outlier_removal.setInputCloud(radius_pcl_ptr);
        radius_outlier_removal.setRadiusSearch(Configurations::get_instance()->ror_radius);
        radius_outlier_removal.setMinNeighborsInRadius(Configurations::get_instance()->ror_min_neighbours);
        radius_outlier_removal.filter(*radius_pcl_ptr);
        std::cout << "Loop: " << loop << " remaining cloud: " << radius_pcl_ptr->points.size() << "\n";
    }
    return radius_pcl_ptr;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_based_filtering_noise(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pcl_ptr) {
    std::cout << "Color-based outliers removal.\n";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_pcl_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_tree_flann;
    kd_tree_flann.setInputCloud(input_pcl_ptr);
    for (size_t i = 0; i < input_pcl_ptr->points.size(); ++i) {
        pcl::PointXYZRGB near_point = input_pcl_ptr->points[i];
        int K = Configurations::get_instance()->cl_based_neighbours;
        double cl_stdev_thresh = Configurations::get_instance()->cl_based_stdev_thresh;
        std::vector<int> k_indices;
        std::vector<float> k_squared_distances;
        if (kd_tree_flann.nearestKSearch(near_point, K, k_indices, k_squared_distances) > 0) {
            pcl::PointXYZRGB rgb_point = near_point;
            float avg_r = 0;
            float avg_g = 0;
            float avg_b = 0;
            for (size_t j = 0; j < K; ++j) {
                avg_r += input_pcl_ptr->points[k_indices[j]].r;
                avg_g += input_pcl_ptr->points[k_indices[j]].g;
                avg_b += input_pcl_ptr->points[k_indices[j]].b;
            }
            avg_r /= K;
            avg_g /= K;
            avg_b /= K;
            float std_dev = 0;
            for (size_t j = 0; j < K; ++j) {
                float d_red = input_pcl_ptr->points[k_indices[j]].r - avg_r;
                float d_green = input_pcl_ptr->points[k_indices[j]].g - avg_g;
                float d_blue = input_pcl_ptr->points[k_indices[j]].b - avg_b;
                float d_rgb_squared = d_red * d_red + d_green * d_green + d_blue * d_blue;
                std_dev += d_rgb_squared;
            }
            std_dev = sqrt(std_dev);
            std_dev /= K;
            float d_red = rgb_point.r - avg_r;
            float d_green = rgb_point.g - avg_g;
            float d_blue = rgb_point.b - avg_b;
            float d_rgb = sqrt(d_red * d_red + d_green * d_green + d_blue * d_blue);
            if (d_rgb / std_dev < cl_stdev_thresh) {
                color_pcl_ptr->points.push_back(rgb_point);
            } else {
            }
        }
    }
    color_pcl_ptr->width = color_pcl_ptr->points.size();
    color_pcl_ptr->height = 1;
    std::cout << "ply_file after color-based filtering: " << *color_pcl_ptr << "\n";
    return color_pcl_ptr;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr stat_color_filtering_noise(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pcl_ptr) {
    std::cout << "Statistical and color-based outliers removal.\n";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr stat_color_pcl_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    stat_color_pcl_ptr = stat_filtering_noise(input_pcl_ptr);
    stat_color_pcl_ptr = color_based_filtering_noise(stat_color_pcl_ptr);
    return stat_color_pcl_ptr;
}

// The down sampling functions
pcl::PointCloud<pcl::PointXYZRGB>::Ptr median_down_sampling(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pcl_ptr) {
    std::cout << "Median down sampling.\n";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr median_pcl_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
    double leaf_size = Configurations::get_instance()->leaf_size;
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.setInputCloud(input_pcl_ptr);
    voxel_grid.filter(*median_pcl_ptr);
    return median_pcl_ptr;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr nearest_down_sampling(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pcl_ptr) {
    std::cout << "Nearest median down sampling.\n";
    double leaf_size = Configurations::get_instance()->leaf_size;
    return nearest_down_sampling_with_leaf_size(input_pcl_ptr, leaf_size);
}

void print_matching_option(const char* parameter_ptr, const int option) {
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
    for (int i = 0; i < option_num; i++) {
        if (parameter.compare(options[i]) == 0) {
            return i;
        }
    }

    std::cout << "not exists " << parameter << " options\n";
    return INVALID;
}

int config_value_by_option(const int option, char* parameter_ptr) {
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
        for (; i < method_num; i++) {
            if (parameter.compare(method_name[i]) == 0) {
                break;
            }
        }
    }

    // There are 3 noise filtering methods
    if (option == 0) {
        if (i > 3)
            goto error;
        command_option.noise = functions[i];
    }

    // There are 2 down sampling methods
    else if (option == 1) {
        if (i <= 3)
            goto error;
        command_option.down_sampling = functions[i];
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
        command_option.input = parameter_ptr;
    } else if (option == 4) {
        command_option.offset = parameter_ptr;
    } else if (option == 5) {
        command_option.inter_noise = parameter_ptr;
    } else if (option == 6) {
        command_option.inter_down_sampling = parameter_ptr;
    }

    return 1;
error:
    print_matching_option(parameter_ptr, option);
    return INVALID;
}

int main(int argc, char* argv[]) {
    int option_index = -1;

    Configurations::get_instance()->read_config();

    // Loop through option parameter
    for (int i = 1; i < argc; i++) {
        if (i % 2 == 1) {
            option_index = get_option(argv[i]);

            // Crash program when param is not exists
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

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr origin_pcl_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

    std::string las_file = command_option.input;
    std::ifstream ifs;
    ifs.open(las_file.c_str(), std::ios::in | std::ios::binary);
    liblas::ReaderFactory f;
    liblas::Reader reader = f.CreateWithStream(ifs);
    liblas::Header const& header = reader.GetHeader();
    std::cout << "las_file size: " << header.GetPointRecordsCount() << "\n";

    double offset_x, offset_y, offset_z;
    offset_x = header.GetMinX();
    offset_y = header.GetMinY();
    offset_z = header.GetMinZ();
    std::ofstream ofs_offset(command_option.offset);
    ofs_offset << std::fixed << offset_x << " " << std::fixed << offset_y << " " << std::fixed << offset_z;
    ofs_offset.close();
    std::cout << "offset_x: " << offset_x << "\n";
    std::cout << "offset_y: " << offset_y << "\n";
    std::cout << "offset_z: " << offset_z << "\n";

    while (reader.ReadNextPoint()) {
        liblas::Point const& point = reader.GetPoint();
        pcl::PointXYZRGB current_point;
        current_point.x = point.GetX() - offset_x;
        current_point.y = point.GetY() - offset_y;
        current_point.z = point.GetZ() - offset_z;
        current_point.b = point.GetColor().GetBlue() / 256;
        current_point.g = point.GetColor().GetGreen() / 256;
        current_point.r = point.GetColor().GetRed() / 256;
        origin_pcl_ptr->points.push_back(current_point);
    }
    origin_pcl_ptr->width = origin_pcl_ptr->points.size();
    origin_pcl_ptr->height = 1;
    std::cout << "ply_file: " << *origin_pcl_ptr << "\n";
    std::cout << "origin_pcl_ptr: " << origin_pcl_ptr->points.size() << "\n";

    // Pre down sample
    std::cout << "Pre down sampling.\n";
    double leaf_size = Configurations::get_instance()->leaf_size / 1.5;
    origin_pcl_ptr = nearest_down_sampling_with_leaf_size(origin_pcl_ptr, leaf_size);
    std::cout << "origin_pcl_ptr after pre down sampling: " << origin_pcl_ptr->points.size() << "\n";

    // Filter noise
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr noise_pcl_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr remaining_pcl_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    remaining_pcl_ptr = command_option.noise(origin_pcl_ptr);
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_remaining;
    kd_remaining.setInputCloud(remaining_pcl_ptr);
    for (size_t i = 0; i < origin_pcl_ptr->points.size(); ++i) {
        pcl::PointXYZRGB this_point = origin_pcl_ptr->points[i];
        std::vector<int> k_indices;
        std::vector<float> k_squared_distances;
        if (kd_remaining.nearestKSearch(this_point, 1, k_indices, k_squared_distances) > 0) {
            if (k_squared_distances[0] > 0) {
                noise_pcl_ptr->points.push_back(this_point);
            }
        }
    }
    noise_pcl_ptr->width = noise_pcl_ptr->points.size();
    noise_pcl_ptr->height = 1;
    std::cout << "p_noise " << noise_pcl_ptr->points.size() << "\n";
    if (command_option.inter) {
        pcl::io::savePLYFile(command_option.inter_noise, *remaining_pcl_ptr, true);
        std::cout << command_option.inter_noise << " saved.\n";
        std::string noise_file = command_option.inter_noise;
        noise_file.insert(noise_file.length() - 4, "_noise");
        pcl::io::savePLYFile(noise_file, *noise_pcl_ptr, true);
        std::cout << noise_file << " saved.\n";
    }
    std::cout << "remaining_pcl_ptr after noise filtering: " << remaining_pcl_ptr->points.size() << "\n";

    // Down sample
    remaining_pcl_ptr = command_option.down_sampling(remaining_pcl_ptr);
    if (command_option.inter) {
        pcl::io::savePLYFile(command_option.inter_down_sampling, *remaining_pcl_ptr, true);
        std::cout << command_option.inter_down_sampling << " saved.\n";
    }
    std::cout << "remaining_pcl_ptr after down sampling: " << remaining_pcl_ptr->points.size() << "\n";
    return 0;
}
