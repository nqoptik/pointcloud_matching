#ifndef CONFIGUTATIONS_HPP
#define CONFIGUTATIONS_HPP

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

class Configurations
{
public:
    Configurations();
    ~Configurations();

    double leaf_size;

    int sor_neighbours;
    double sor_stdev_thresh;
    int sor_iterations;

    double ror_radius;
    int ror_min_neighbours;
    int ror_iterations;

    size_t cl_based_neighbours;
    double cl_based_stdev_thresh;
    int cl_based_iterations;

    double iss_old_salient_radius;
    double iss_old_nonmax_radius;
    int iss_old_min_neighbours;
    double iss_new_salient_radius;
    double iss_new_nonmax_radius;
    int iss_new_min_neighbours;

    double susan_old_radius;
    double susan_new_radius;

    double harris3d_radius;

    double icp_radius;
    int icp_iterations;
    double icp_refine_radius;

    double shot_radius;

    double fpfh_radius;

    float OF_error_threshold;
    double pi_theta_x;
    double pi_theta_y;

    double pos_radius;
    bool draw_old_kpts;
    bool draw_new_kpts;
    bool draw_old_colour;
    bool draw_new_colour;

    void read_config();

    static Configurations* get_instance()
    {
        return instance_;
    }

private:
    static Configurations* instance_;
};

#endif // CONFIGUTATIONS_HPP
