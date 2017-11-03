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

	void readConfig();

	static Configurations* getInstance() {
		return _instance;
	}

private:

	static Configurations* _instance;
};

#endif /* _CONFIGUTATIONS_H_ */
