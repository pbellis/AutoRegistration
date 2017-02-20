#ifndef STATISTICAL_SHAPE_H
#define STATISTICAL_SHAPE_H

#include "alignmentpredictionlib_global.h"

#include <vector>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

// Statistical Shape
// See: http://www.isprs-ann-photogramm-remote-sens-spatial-inf-sci.net/II-3/9/2014/isprsannals-II-3-9-2014.pdf

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

//!
//! \brief The StatisticalShape struct
//!     A structure that contains various statistical characteristics of a point in a point cloud.
//!     See: http://www.isprs-ann-photogramm-remote-sens-spatial-inf-sci.net/II-3/9/2014/isprsannals-II-3-9-2014.pdf
//!
struct StatisticalShape
{
    double linearity;
    double planarity;
    double sphericity;
    double cubed_omnivariance;
    double anisotropy;
    double curvature_change;

    // THE MOST IMPORTANT FEATURES ARE
    // linearity, planarity, sphericity, curvature_change
    // WHEN curvature_change IS HIGH, POINT IS LIKELY TO BE KEYPOINT
    // WHEN linearity, planarity, sphericity ARE HIGH, POINT IS LIKELY TO NOT BE A KEYPOINT
};

//!
//! \brief calcuate_statistical_shape
//!     A function that calculates the statistical characteristics per point. Running time is O(n * log(n).
//! \param statistical_shape
//!     The statistical characteristics of each point
//! \param cloud
//!     The point cloud to perform calculations on
//! \param kdtree
//!     The search algorthm for finding neighboring points
//! \param radius
//!     The max distance for a point to be considered as another point's neighbor.
//!
void ALIGNMENTPREDICTIONLIBSHARED_EXPORT calcuate_statistical_shape(std::vector<StatisticalShape> &statistical_shape, const pcl::PointCloud<pcl::PointXYZ> &cloud, const pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree, double radius);

//!
//! \brief calcuate_statistical_shape
//!     A function that calculates the statistical characteristics per point. Running time is O(n * log(n).
//! \param statistical_shape
//!     The statistical characteristics of each point
//! \param cloud
//!     The point cloud to perform calculations on
//! \param kdtree
//!     The search algorthm for finding neighboring points
//! \param radius
//!     The max distance for a point to be considered as another point's neighbor.
//! \param indices
//!     A list of indices to use.
//!
void ALIGNMENTPREDICTIONLIBSHARED_EXPORT calcuate_statistical_shape(std::vector<StatisticalShape> &statistical_shape, const pcl::PointCloud<pcl::PointXYZ> &cloud, const pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree, double radius, const std::vector<int> &indices);

#endif // STATISTICAL_SHAPE_H
