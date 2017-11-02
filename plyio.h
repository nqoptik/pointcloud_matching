#ifndef _PLYIO_H_
#define _PLYIO_H_

#include <iostream>
#include <sstream>
#include <vector>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

void savePly(std::string fileName, std::vector<pcl::PointXYZRGB> listxyzrgb);

#endif /* _PLYIO_H_ */

