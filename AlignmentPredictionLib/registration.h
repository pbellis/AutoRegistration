#ifndef ALIGNMENT_H
#define ALIGNMENT_H

#include "alignmentpredictionlib_global.h"

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include "mean.h"
#include "covariance.h"
#include "graph_theory.h"
#include "statistical_distance.h"

bool ALIGNMENTPREDICTIONLIBSHARED_EXPORT globalized_alignment(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_vec, pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr &icp, int iters);

#endif // ALIGNMENT_H
