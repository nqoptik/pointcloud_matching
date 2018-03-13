#pragma once

#ifndef _CLOUDDIFFCHECKER_H_
#define _CLOUDDIFFCHECKER_H_

#include <iostream>
#include <vector>
#include <algorithm>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml/ml.hpp>

struct PlaneCoefficients {

	float o_a, o_b, o_c, o_d, o_inliers, o_total;
	float n_a, n_b, n_c, n_d, n_inliers, n_total;

	PlaneCoefficients(float o_a = 0, float o_b = 0, float o_c = 0, float o_d = 0, float o_inliers = 0, float o_total = 0,
		float n_a = 0, float n_b = 0, float n_c = 0, float n_d = 0, float n_inliers = 0, float n_total = 0) {

		this->o_a = o_a;
		this->o_b = o_b;
		this->o_c = o_c;
		this->o_d = o_d;
		this->o_inliers = o_inliers;
		this->o_total = o_total;
		this->n_a = n_a;
		this->n_b = n_b;
		this->n_c = n_c;
		this->n_d = n_d;
		this->n_inliers = n_inliers;
		this->n_total = n_total;
	}
};

struct ReferPlane {

	float r_a, r_b, r_c, r_d;

	ReferPlane(float r_a = 0, float r_b = 0, float r_c = 0, float r_d = 0) {
		this->r_a = r_a;
		this->r_b = r_b;
		this->r_c = r_c;
		this->r_d = r_d;
	}
};

class CloudDiffChecker {

private:

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pOld;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pNew;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pOldDiff;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pNewDiff;
	std::vector<PlaneCoefficients> planeCoefficients;
	std::vector<int> clusterIndices;
	std::vector<ReferPlane> referPlanes;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pOldProj;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pNewProj;
	double old_res, new_res;
	float min_moving_distance;
	float x_min, x_max;
	float y_min, y_max;
	float z_min, z_max;
	float d_max;
	int grid_count;
	int min_points_in_grid;
	int x_grid_count, y_grid_count;
	float grid_step_length;
	float ransacDistanceThreshold;

public:

	CloudDiffChecker(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pOld, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pNew);
	~CloudDiffChecker();

	static double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud);
	void determineDiffRegions();
	void getCloudParameters();
	void griddingDiff();
	void getReferPlane();
	void getProjections();
	void drawTransformation();
	static float distance(pcl::PointXYZRGB p1, pcl::PointXYZRGB p2);
	static float squaredDistance(pcl::PointXYZRGB p1, pcl::PointXYZRGB p2);
	static pcl::PointXYZRGB projectionOntoPlane(pcl::PointXYZRGB project_point, pcl::PointXYZRGB plane_point, pcl::PointXYZRGB plane_normal);
	static pcl::PointXYZRGB lineOntoPlane(pcl::PointXYZRGB point, pcl::PointXYZRGB normal, float A, float B, float C, float D);
};

#endif /* _CLOUDDIFFCHECKER_H_ */
