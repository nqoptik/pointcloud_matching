#include "pointcloud_matching/configurations.hpp"

Configurations* Configurations::instance_ = new Configurations();

Configurations::Configurations() {
    leaf_size = 0.001;

    sor_neighbours = 50;
    sor_stdev_thresh = 1.0;
    sor_iterations = 5;

    ror_radius = 0.003;
    ror_min_neighbours = 10;
    ror_iterations = 5;

    cl_based_neighbours = 50;
    cl_based_stdev_thresh = 10.0;
    cl_based_iterations = 5;

    iss_old_salient_radius = 0.05;
    iss_old_nonmax_radius = 0.04;
    iss_old_min_neighbours = 10;
    iss_new_salient_radius = 0.005;
    iss_new_nonmax_radius = 0.004;
    iss_new_min_neighbours = 1;

    susan_old_radius = 0.04;
    susan_new_radius = 0.04;

    harris3d_radius = 0.04;

    icp_radius = 0.05;
    icp_iterations = 5;
    icp_refine_radius = 0.005;

    shot_radius = 0.05;

    fpfh_radius = 0.02;

    OF_error_threshold = 20;
    pi_theta_x = 0.0;
    pi_theta_y = 0.0;

    pos_radius = 0.03;

    draw_old_colour = false;
    draw_new_colour = false;
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
        } else if (option.compare("sor_neighbours") == 0) {
            int val;
            in >> val;
            sor_neighbours = val;
        } else if (option.compare("sor_stdev_thresh") == 0) {
            double val;
            in >> val;
            sor_stdev_thresh = val;
        } else if (option.compare("sor_iterations") == 0) {
            int val;
            in >> val;
            sor_iterations = val;
        } else if (option.compare("ror_radius") == 0) {
            double val;
            in >> val;
            ror_radius = val;
        } else if (option.compare("ror_min_neighbours") == 0) {
            int val;
            in >> val;
            ror_min_neighbours = val;
        } else if (option.compare("ror_iterations") == 0) {
            int val;
            in >> val;
            ror_iterations = val;
        } else if (option.compare("cl_based_neighbours") == 0) {
            int val;
            in >> val;
            cl_based_neighbours = val;
        } else if (option.compare("cl_based_stdev_thresh") == 0) {
            double val;
            in >> val;
            cl_based_stdev_thresh = val;
        } else if (option.compare("cl_based_iterations") == 0) {
            int val;
            in >> val;
            cl_based_iterations = val;
        } else if (option.compare("iss_old_salient_radius") == 0) {
            double val;
            in >> val;
            iss_old_salient_radius = val;
        } else if (option.compare("iss_old_nonmax_radius") == 0) {
            double val;
            in >> val;
            iss_old_nonmax_radius = val;
        } else if (option.compare("iss_old_min_neighbours") == 0) {
            int val;
            in >> val;
            iss_old_min_neighbours = val;
        } else if (option.compare("iss_new_salient_radius") == 0) {
            double val;
            in >> val;
            iss_new_salient_radius = val;
        } else if (option.compare("iss_new_nonmax_radius") == 0) {
            double val;
            in >> val;
            iss_new_nonmax_radius = val;
        } else if (option.compare("iss_new_min_neighbours") == 0) {
            int val;
            in >> val;
            iss_new_min_neighbours = val;
        } else if (option.compare("susan_old_radius") == 0) {
            double val;
            in >> val;
            susan_old_radius = val;
        } else if (option.compare("susan_new_radius") == 0) {
            double val;
            in >> val;
            susan_new_radius = val;
        } else if (option.compare("harris3d_radius") == 0) {
            double val;
            in >> val;
            harris3d_radius = val;
        } else if (option.compare("icp_radius") == 0) {
            double val;
            in >> val;
            icp_radius = val;
        } else if (option.compare("icp_iterations") == 0) {
            int val;
            in >> val;
            icp_iterations = val;
        } else if (option.compare("icp_refine_radius") == 0) {
            double val;
            in >> val;
            icp_refine_radius = val;
        } else if (option.compare("shot_radius") == 0) {
            double val;
            in >> val;
            shot_radius = val;
        } else if (option.compare("fpfh_radius") == 0) {
            double val;
            in >> val;
            fpfh_radius = val;
        } else if (option.compare("OF_error_threshold") == 0) {
            float val;
            in >> val;
            OF_error_threshold = val;
        } else if (option.compare("pi_theta_x") == 0) {
            double val;
            in >> val;
            pi_theta_x = val;
        } else if (option.compare("pi_theta_y") == 0) {
            double val;
            in >> val;
            pi_theta_y = val;
        } else if (option.compare("pos_radius") == 0) {
            double val;
            in >> val;
            pos_radius = val;
        } else if (option.compare("draw_old_colour") == 0) {
            bool val;
            in >> val;
            draw_old_colour = val;
        } else if (option.compare("draw_new_colour") == 0) {
            bool val;
            in >> val;
            draw_new_colour = val;
        }
    }
    ifs.close();

    // Print configurations
    std::cout << "leaf_size:              " << leaf_size << "\n";
    std::cout << "sor_neighbours:         " << sor_neighbours << "\n";
    std::cout << "sor_stdev_thresh:       " << sor_stdev_thresh << "\n";
    std::cout << "sor_iterations:         " << sor_iterations << "\n";
    std::cout << "ror_radius:             " << ror_radius << "\n";
    std::cout << "ror_min_neighbours:     " << ror_min_neighbours << "\n";
    std::cout << "ror_iterations:         " << ror_iterations << "\n";
    std::cout << "cl_based_neighbours:    " << cl_based_neighbours << "\n";
    std::cout << "cl_based_stdev_thresh:  " << cl_based_stdev_thresh << "\n";
    std::cout << "cl_based_iterations:    " << cl_based_iterations << "\n";
    std::cout << "iss_old_salient_radius: " << iss_old_salient_radius << "\n";
    std::cout << "iss_old_nonmax_radius:  " << iss_old_nonmax_radius << "\n";
    std::cout << "iss_old_min_neighbours: " << iss_old_min_neighbours << "\n";
    std::cout << "iss_new_salient_radius: " << iss_new_salient_radius << "\n";
    std::cout << "iss_new_nonmax_radius:  " << iss_new_nonmax_radius << "\n";
    std::cout << "iss_new_min_neighbours: " << iss_new_min_neighbours << "\n";
    std::cout << "susan_old_radius:       " << susan_old_radius << "\n";
    std::cout << "susan_new_radius:       " << susan_new_radius << "\n";
    std::cout << "harris3d_radius:        " << harris3d_radius << "\n";
    std::cout << "icp_radius:             " << icp_radius << "\n";
    std::cout << "icp_iterations:         " << icp_iterations << "\n";
    std::cout << "icp_refine_radius:      " << icp_refine_radius << "\n";
    std::cout << "shot_radius:            " << shot_radius << "\n";
    std::cout << "fpfh_radius:            " << fpfh_radius << "\n";
    std::cout << "OF_error_threshold:     " << OF_error_threshold << "\n";
    std::cout << "pi_theta_x:             " << pi_theta_x << "\n";
    std::cout << "pi_theta_y:             " << pi_theta_y << "\n";
    std::cout << "pos_radius:             " << pos_radius << "\n";
    std::cout << "draw_old_colour:        " << draw_old_colour << "\n";
    std::cout << "draw_new_colour:        " << draw_new_colour << "\n";
}
