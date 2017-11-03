#include "Configurations.h"

Configurations* Configurations::_instance = new Configurations();

Configurations::Configurations() {

    leaf_size = 0.001;
    sor_neighbours = 50;
    sor_stdev_thresh = 1.0;
    iss_old_salient_radius = 0.05;
    iss_old_nonmax_radius = 0.04;
    iss_old_min_neighbours = 10;
    iss_new_salient_radius = 0.005;
    iss_new_nonmax_radius = 0.004;
    iss_new_min_neighbours = 1;
}

Configurations::~Configurations() {
}

void Configurations::readConfig() {

    std::ifstream ifs("config.ini");
    std::string line;
    while (std::getline(ifs, line)) {
        std::istringstream in(line);
        std::string option;
        in >> option;

        if (option.compare("leaf_size") == 0) {
            double val;
            in >> val;
            leaf_size = val;
        }
        else if (option.compare("sor_neighbours") == 0) {
            int val;
            in >> val;
            sor_neighbours = val;
        }
        else if (option.compare("sor_stdev_thresh") == 0) {
            double val;
            in >> val;
            sor_stdev_thresh = val;
        }
        else if (option.compare("iss_old_salient_radius") == 0) {
            double val;
            in >> val;
            iss_old_salient_radius = val;
        }
        else if (option.compare("iss_old_nonmax_radius") == 0) {
            double val;
            in >> val;
            iss_old_nonmax_radius = val;
        }
        else if (option.compare("iss_old_min_neighbours") == 0) {
            int val;
            in >> val;
            iss_old_min_neighbours = val;
        }
        else if (option.compare("iss_new_salient_radius") == 0) {
            double val;
            in >> val;
            iss_new_salient_radius = val;
        }
        else if (option.compare("iss_new_nonmax_radius") == 0) {
            double val;
            in >> val;
            iss_new_nonmax_radius = val;
        }
        else if (option.compare("iss_new_min_neighbours") == 0) {
            int val;
            in >> val;
            iss_new_min_neighbours = val;
        }
    }
    ifs.close();

    // Print configurations
    std::cout << "leaf_size:              " << leaf_size << "\n";
    std::cout << "sor_neighbours:         " << sor_neighbours << "\n";
    std::cout << "sor_stdev_thresh:       " << sor_stdev_thresh << "\n";
    std::cout << "iss_old_salient_radius: " << iss_old_salient_radius << "\n";
    std::cout << "iss_old_nonmax_radius:  " << iss_old_nonmax_radius << "\n";
    std::cout << "iss_old_min_neighbours: " << iss_old_min_neighbours << "\n";
    std::cout << "iss_new_salient_radius: " << iss_new_salient_radius << "\n";
    std::cout << "iss_new_nonmax_radius:  " << iss_new_nonmax_radius << "\n";
    std::cout << "iss_new_min_neighbours: " << iss_new_min_neighbours << "\n";
}
