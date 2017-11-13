#pragma once

#ifndef _CONFIGUTATIONS_H_
#define _CONFIGUTATIONS_H_

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

class Configurations {

public:

    Configurations();
    ~Configurations();

    double leaf_size;
    int sor_neighbours;
    double sor_stdev_thresh;
    double iss_old_salient_radius;
    double iss_old_nonmax_radius;
    int iss_old_min_neighbours;
    double iss_new_salient_radius;
    double iss_new_nonmax_radius;
    int iss_new_min_neighbours;
    double des_radius;
    double pos_radius;
    int icp_iterations;
    double refine_radius;
    bool draw_old_kpts;
    bool draw_new_kpts;
    bool draw_old_colour;
    bool draw_new_colour;

	void readConfig();

	static Configurations* getInstance() {
		return _instance;
	}

private:

	static Configurations* _instance;
};

#endif /* _CONFIGUTATIONS_H_ */
