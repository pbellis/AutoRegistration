#ifndef STATISTICAL_SHAPE_H
#define STATISTICAL_SHAPE_H

#include "alignmentpredictionlib_global.h"

#include <vector>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

//!
//! \brief calculate_ownership_error
//!     A function that calculates the uncertainty that a point can be classified to one of two distributions. In other words a point in the overlapping regions
//!     of two clouds will have an equal probability of belonging to each cloud meaning that we are uncertain as to which cloud they belong to. Runnign time is O(n).
//!     See: https://en.wikipedia.org/wiki/Bayesian_statistics
//! \param source_entropy
//!     The resulting entropy of each point in the source cloud
//! \param source_mean
//!     The mean (or centroid) of the source cloud.
//! \param source_covariance
//!     The covariance matrix of the source cloud.
//! \param source_cloud
//!     The source cloud
//! \param target_mean
//!     The mean (or centroid) of the target cloud
//! \param target_covariance
//!     The covariance matrix of the target cloud
//! \param target_size
//!     The number of points belonging to the target cloud
//!
void ALIGNMENTPREDICTIONLIBSHARED_EXPORT calculate_ownership_uncertainty(std::vector<double> &source_ownership_uncertainty, const Eigen::Vector3d &source_mean, const Eigen::Matrix3d &source_covariance, const pcl::PointCloud<pcl::PointXYZ> &source_cloud, const Eigen::Vector3d &target_mean, const Eigen::Matrix3d &target_covariance, size_t target_size);

//!
//! \brief calculate_ownership_error
//!     A function that calculates the uncertainty that a point can be classified to one of two distributions. In other words a point in the overlapping regions
//!     of two clouds will have an equal probability of belonging to each cloud meaning that we are uncertain as to which cloud they belong to. Runnign time is O(n).
//!     See: https://en.wikipedia.org/wiki/Bayesian_statistics
//! \param source_entropy
//!     The resulting entropy of each point in the source cloud
//! \param source_mean
//!     The mean (or centroid) of the source cloud.
//! \param source_covariance
//!     The covariance matrix of the source cloud.
//! \param source_cloud
//!     The source cloud
//! \param source_indices
//!     A list of indices to use.
//! \param target_mean
//!     The mean (or centroid) of the target cloud
//! \param target_covariance
//!     The covariance matrix of the target cloud
//! \param target_size
//!     The number of points belonging to the target cloud
//!
void ALIGNMENTPREDICTIONLIBSHARED_EXPORT calculate_ownership_uncertainty(std::vector<double> &source_ownership_uncertainty, const Eigen::Vector3d &source_mean, const Eigen::Matrix3d &source_covariance, const pcl::PointCloud<pcl::PointXYZ> &source_cloud, const std::vector<int> &source_indices, const Eigen::Vector3d &target_mean, const Eigen::Matrix3d &target_covariance, size_t target_size);

void ALIGNMENTPREDICTIONLIBSHARED_EXPORT find_uncertain_region(std::vector<int> &uncertain_indices, const std::vector<double> &ownership_uncertainty, const pcl::PointCloud<pcl::PointXYZ> &cloud, double total_uncertainty_probability);

void ALIGNMENTPREDICTIONLIBSHARED_EXPORT calcuate_curvature_change(std::vector<double> &curvature_change, const pcl::PointCloud<pcl::PointXYZ> &cloud, const pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree, double radius);

//!     See: http://www.isprs-ann-photogramm-remote-sens-spatial-inf-sci.net/II-3/9/2014/isprsannals-II-3-9-2014.pdf
void ALIGNMENTPREDICTIONLIBSHARED_EXPORT calculate_statistical_shape(Eigen::MatrixXd &statistical_shape, const pcl::PointCloud<pcl::PointXYZ> &cloud, const pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree, double radius);

void ALIGNMENTPREDICTIONLIBSHARED_EXPORT calculate_statistical_shape(Eigen::MatrixXd &statistical_shape, const pcl::PointCloud<pcl::PointXYZ> &cloud, const pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree, int k);

void ALIGNMENTPREDICTIONLIBSHARED_EXPORT calculate_features(Eigen::VectorXd &features, const Eigen::MatrixXd &statistical_shape, double det, const Eigen::VectorXd &mean, const Eigen::MatrixXd &precision);

#endif // STATISTICAL_SHAPE_H
