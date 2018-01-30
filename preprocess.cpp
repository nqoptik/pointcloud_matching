#include "preprocess.h"
#include "Configurations.h"

/* noise filtering methods*/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr statFilteringNoise(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_stat_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (int loop = 0; loop < 1; ++loop) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud (p);
        sor.setMeanK (Configurations::getInstance()->sor_neighbours);
        sor.setStddevMulThresh (Configurations::getInstance()->sor_stdev_thresh);
        sor.filter (*p_stat_pcl);
        std::cout << "Loop: " << loop << " remaining cloud: " << p_stat_pcl->points.size() << "\n";
    }
    for (int loop = 1; loop < 5; ++loop) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud (p_stat_pcl);
        sor.setMeanK (Configurations::getInstance()->sor_neighbours);
        sor.setStddevMulThresh (Configurations::getInstance()->sor_stdev_thresh);
        sor.filter (*p_stat_pcl);
        std::cout << "Loop: " << loop << " remaining cloud: " << p_stat_pcl->points.size() << "\n";
    }
    return p_stat_pcl;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr radiusFilteringNoise(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_radius_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (int loop = 0; loop < 1; ++loop) {
        pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
        ror.setInputCloud(p);
        ror.setRadiusSearch(Configurations::getInstance()->leaf_size*3);
        ror.setMinNeighborsInRadius (10);
        ror.filter (*p_radius_pcl);
        std::cout << "Loop: " << loop << " remaining cloud: " << p_radius_pcl->points.size() << "\n";
    }
    for (int loop = 1; loop < 5; ++loop) {
        pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
        ror.setInputCloud(p_radius_pcl);
        ror.setRadiusSearch(Configurations::getInstance()->leaf_size*3);
        ror.setMinNeighborsInRadius (10);
        ror.filter (*p_radius_pcl);
        std::cout << "Loop: " << loop << " remaining cloud: " << p_radius_pcl->points.size() << "\n";
    }
    return p_radius_pcl;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorbasedFilteringNoise(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_color_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_p;
    kd_p.setInputCloud(p);
    for (size_t i = 0; i < p->points.size(); ++i) {
        pcl::PointXYZRGB nearP = p->points[i];
        int K = Configurations::getInstance()->sor_neighbours;
        std::vector<int> pointIdxNKNSearch;
        std::vector<float> pointNKNSquaredDistance;
        if (kd_p.nearestKSearch (nearP, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            pcl::PointXYZRGB colorP = nearP;
            float avg_r = 0;
            float avg_g = 0;
            float avg_b = 0;
            for (size_t j = 0; j < K; ++j) {
                avg_r += p->points[pointIdxNKNSearch[j]].r;
                avg_g += p->points[pointIdxNKNSearch[j]].g;
                avg_b += p->points[pointIdxNKNSearch[j]].b;
            }
            avg_r /= K;
            avg_g /= K;
            avg_b /= K;
            float std_dev = 0;
            for (size_t j = 0; j < K; ++j) {
                float dr = p->points[pointIdxNKNSearch[j]].r - avg_r;
                float dg = p->points[pointIdxNKNSearch[j]].g - avg_g;
                float db = p->points[pointIdxNKNSearch[j]].b - avg_b;
                float d_color = dr*dr + dg*dg + db*db;
                std_dev += d_color;
            }
            std_dev = sqrt(std_dev);
            std_dev /= K;
            float dr_color = colorP.r - avg_r;
            float dg_color = colorP.g - avg_g;
            float db_color = colorP.b - avg_b;
            float d_colorP = sqrt(dr_color*dr_color + dg_color*dg_color + db_color*db_color);
            if (d_colorP/std_dev < 10) {
                p_color_pcl->points.push_back(colorP);
            }
            else {
            }
        }
    }
    p_color_pcl->width = p_color_pcl->points.size();
    p_color_pcl->height = 1;
    std::cout << "ply_file after color-based filtering: " << *p_color_pcl << "\n";
    return p_color_pcl;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr bilateralFilteringNoise(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p) {

}

/* Down Sampling */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr downSamplingVirtualCenter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_real_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::VoxelGrid<pcl::PointXYZRGB> grid;
    double leaf = Configurations::getInstance()->leaf_size;
    grid.setLeafSize(leaf, leaf, leaf);
    grid.setInputCloud(p);
    grid.filter(*p_real_pcl);
    return p_real_pcl;

    // Down sampling with real point near center
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_p;
    kd_p.setInputCloud(p);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_vir_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (size_t i = 0; i < p_real_pcl->points.size(); ++i) {
        pcl::PointXYZRGB centerP = p_real_pcl->points[i];
        std::vector<int> pointIdxNKNSearch;
        std::vector<float> pointNKNSquaredDistance;
        if (kd_p.nearestKSearch (centerP, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            if (pointNKNSquaredDistance[0] < leaf*leaf) {
                p_vir_pcl->points.push_back(p->points[pointIdxNKNSearch[0]]);
            }
        }
    }
    if (p_vir_pcl->points.size() != p_real_pcl->points.size()) {
        std::cout << "Need to check down sampling with real piont center.\n";
        exit(1);
    }
    p_vir_pcl->width = p_vir_pcl->points.size();
    p_vir_pcl->height = 1;
    return p_vir_pcl;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr downSamplingRealCenter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_real_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::VoxelGrid<pcl::PointXYZRGB> grid;
    double leaf = Configurations::getInstance()->leaf_size;
    grid.setLeafSize(leaf, leaf, leaf);
    grid.setInputCloud(p);
    grid.filter(*p_real_pcl);
    return p_real_pcl;
}

void PrintMatchingOption (char* para, int option) {
    if (para == NULL || option >= sizeof(para)/sizeof(para[0])) {
        std::cout << ERROR << "\n";
        std::cout << HELP << "\n";
        return;
    }

    std::cout << "not exists " << para << " for " << options[option] << "\n";
}

int getOption (char* _p) {
    if (_p == NULL) {
        std::cout << ERROR << "\n";
        std::cout << HELP << "\n";
        return UNVALID;
    }

    std::string para(_p);
    int optionNum = sizeof(options)/sizeof(options[0]);
    // loop and compare
    for (int i = 0; i < optionNum; i++) {
        if (para.compare(options[i]) == 0) {
            return i;
        }
    }

    std::cout << "not exists " << para << " options\n";
    return UNVALID;
}


int configValueByOption(int option, char* _p) {
        std::string para(_p);

        if (_p == NULL  || para.compare("") == 0 || option >= sizeof(options)/sizeof(options[0])) {
            std::cout << ERROR << "\n";
            std::cout << HELP << "\n";
            return UNVALID;
	}
	
	// loop and compare
        int i = 0;
        if (option < 2) {
            int methodNum = sizeof(methodName)/sizeof(methodName[0]);
            for (; i < methodNum; i++) {
                if (para.compare(methodName[i]) == 0) {
                    break;
                }
            }
        }

        // noise, have 3 method
        if (option == 0) {
            if (i > 3)
                goto error;
            commandOption.noise = functions[i];
        }
        // down sample, have two method
        else if (option == 1) {
            if (i <= 3)
                goto error;
            commandOption.down_sample = functions[i];
        }
        else if (option == 2)
        {
            // Yes or No
            if (para.compare("Y") == 0 || para.compare("y") == 0 ||
                    para.compare("Yes") == 0 || para.compare("yes") == 0) {
                commandOption.inter = true;
            }
            else if (para.compare("N") == 0 || para.compare("n") == 0 ||
                    para.compare("No") == 0 || para.compare("no") == 0) {
                commandOption.inter = false;
            }
            else
                goto error;
        }
        else if (option == 3) {
             commandOption.input = _p;
        }
        else if (option == 4) {
            commandOption.output = _p;
        }

        return 1;
error:
        PrintMatchingOption(_p, option);
	return UNVALID;
}

int main (int argc, char* argv[]) {
	int optionIndex = -1;
	
    Configurations::getInstance()->readConfig();
	// loop thought option parameter
        for (int i = 1; i < argc; i++) {
            if (i % 2 == 1) {
                optionIndex = getOption (argv[i]);

                // crash program when param is not exists
                if (optionIndex == -1) {
                    std::cout << ERROR << "\n";
                    std::cout << HELP << "\n";
                    return UNVALID;
                }
            }
            else {
                if (optionIndex == -1 || configValueByOption(optionIndex, argv[i]) == -1) {
                    std::cout << ERROR << "\n";
                    std::cout << HELP << "\n";
                    return UNVALID;
                }

                // reset
                optionIndex = -1;
            }
	}

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_orgCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    std::string las_file = commandOption.input;
    std::ifstream ifs;
    ifs.open(las_file.c_str(), std::ios::in | std::ios::binary);
    liblas::ReaderFactory f;
    liblas::Reader reader = f.CreateWithStream(ifs);
    liblas::Header const& header = reader.GetHeader();
    std::cout << "las_file size: " << header.GetPointRecordsCount() << "\n";

    double offset_x = Configurations::getInstance()->offset_x;
    double offset_y = Configurations::getInstance()->offset_y;
    double offset_z = Configurations::getInstance()->offset_z;
    while (reader.ReadNextPoint()) {

        liblas::Point const& p = reader.GetPoint();
        pcl::PointXYZRGB cur_point;
        cur_point.x = p.GetX() - offset_x;
        cur_point.y = p.GetY() - offset_y;
        cur_point.z = p.GetZ() - offset_z;
        cur_point.b = p.GetColor().GetBlue()/256;
        cur_point.g = p.GetColor().GetGreen()/256;
        cur_point.r = p.GetColor().GetRed()/256;
        p_orgCloud->points.push_back(cur_point);
    }
    p_orgCloud->width = p_orgCloud->points.size();
    p_orgCloud->height = 1;
    std::cout << "ply_file : " << *p_orgCloud << "\n";
    std::cout << "p_orgCloud " << p_orgCloud->points.size() << "\n";
    p_orgCloud = commandOption.down_sample(p_orgCloud);
    std::cout << "p_orgCloud " << p_orgCloud->points.size() << "\n";
    p_orgCloud = commandOption.noise(p_orgCloud);
    std::cout << "p_orgCloud " << p_orgCloud->points.size() << "\n";
    return 0;
	//todo
}
