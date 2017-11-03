#include "Configurations.h"

Configurations* Configurations::_instance = new Configurations();

Configurations::Configurations() {

    leaf_size = 0.001;
    sor_neighbours = 50;
    sor_stdev_thresh = 1.0;
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
    }
    ifs.close();

    // Print configurations
    std::cout << "leaf_size:        " << leaf_size << "\n";
    std::cout << "sor_neighbours:   " << sor_neighbours << "\n";
    std::cout << "sor_stdev_thresh: " << sor_stdev_thresh << "\n";
}
