#ifndef STATISTICAL_DISTANCE_H
#define STATISTICAL_DISTANCE_H

#include "alignmentpredictionlib_global.h"

#include <vector>
#include <Eigen/Dense>

// Statistical Distance
// Assumes that all point clouds have roughly a multivariate normal distribution that we may compare using the hellinger distance.
// See: https://en.wikipedia.org/wiki/Hellinger_distance

//!
//! \brief calculate_multivariate_hellinger_distance
//!     A function that calculates the statistical distance between two distributions. For the purpose of alignment prediction, we assume that the distribution
//!     is roughly normal. Running time is O(1)
//!     See: https://en.wikipedia.org/wiki/Hellinger_distance
//! \param distance
//!     The statistical distance between two distributions. This in the range [0, 1] where 0 indicates clossness and 1 indicates farness.
//! \param source_mean
//!     The mean (or centroid) of the source cloud
//! \param source_covariance
//!     The covariance matrix of the source cloud
//! \param target_mean
//!     The mean (or centroid) of the target cloud
//! \param target_covariance
//!     The covariance matrix of the target cloud
//!
void ALIGNMENTPREDICTIONLIBSHARED_EXPORT calculate_multivariate_hellinger_distance(double &distance, const Eigen::Vector3d &source_mean, const Eigen::Matrix3d &source_covariance, const Eigen::Vector3d &target_mean, const Eigen::Matrix3d &target_covariance);

//!
//! \brief calculate_multivariate_hellinger_distance
//!     A function that calculates all pairwise statistical distances in a set of N distributions. Each pairwise distance is computed using the above function.
//!     Running time is O(n^2).
//! \param distance_matrix
//!     A distance matrix where distance_matrix(i, j) is the statistical distance between a distribution i and j.
//! \param means
//!     A set of means where means[i] is the mean of distribution i
//! \param covariances
//!     A set of covariances where covariances[i] is the covariance of distribution i
//!
void ALIGNMENTPREDICTIONLIBSHARED_EXPORT calculate_multivariate_hellinger_distance(Eigen::MatrixXd &distance_matrix, const std::vector<Eigen::Vector3d> &means, const std::vector<Eigen::Matrix3d> &covariances);

#endif // STATISTICAL_DISTANCE_H
