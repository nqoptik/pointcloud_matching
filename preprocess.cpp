#include "preprocess.h"
#include "Configurations.h"

/* noise filtering methods*/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr statFilteringNoise(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_stat_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
    // Create the filtering object
    for (int loop = 0; loop < 5; ++loop) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud (p);
        sor.setMeanK (Configurations::getInstance()->sor_neighbours);
        sor.setStddevMulThresh (Configurations::getInstance()->sor_stdev_thresh);
        sor.filter (*p_stat_pcl);
        std::cout << "Loop: " << loop << " ply_file after filtering: " << p_stat_pcl->points.size() << "\n";
    }
    return p_stat_pcl;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr radiusFilteringNoise(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p) {

}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorbasedFilteringNoise(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p) {

}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr bilateralFilteringNoise(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p) {

}

/* Down Sampling */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr downSamplingVirtualCenter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p) {

}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr downSamplingRealCenter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p) {

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
    p_orgCloud = commandOption.noise(p_orgCloud);
    std::cout << "p_orgCloud " << p_orgCloud->points.size() << "\n";
	//todo
}
