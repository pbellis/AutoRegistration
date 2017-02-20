#ifndef MULTIVARIATE_STATISTICS_H
#define MULTIVARIATE_STATISTICS_H

#include "alignmentpredictionlib_global.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include <vector>
#include <algorithm>
#include <numeric>

#include <Eigen/Dense>

// Multivariate Normal Distribution
// See: https://en.wikipedia.org/wiki/Multivariate_normal_distribution

//!
//! \brief calculate_mean
//!     A function that calculates the mean value (or centroid) of a given point cloud. Running time is O(n).
//! \param mean
//!     The mean value (or centroid) of the point cloud
//! \param matrix_map
//!     A 3xN matrix map that correspondes to some point cloud. To get this type call pcl::PointCloud<PointT>::getMatrixXfMap.
//!     See: http://docs.pointclouds.org/trunk/classpcl_1_1_point_cloud.html#ab9486898ac363e8206829da7c01ea62d
//!
template <class Derived>
void calculate_mean(Eigen::Vector3d &mean, const Eigen::MatrixBase<Derived> &matrix_map)
{
    mean = matrix_map.rowwise().mean();
}

//!
//! \brief calculate_mean
//!     A function that calculates the mean value (or centroid) of a given point cloud. Running time is O(n).
//! \param mean
//!     The mean value (or centroid) of the point cloud
//! \param matrix_map
//!     A 3xN matrix map that correspondes to some point cloud. To get this type call pcl::PointCloud<PointT>::getMatrixXfMap.
//!     See: http://docs.pointclouds.org/trunk/classpcl_1_1_point_cloud.html#ab9486898ac363e8206829da7c01ea62d
//! \param indices
//!     A list of indices to use.
//!
template <class Derived>
void calculate_mean(Eigen::Vector3d &mean, const Eigen::MatrixBase<Derived> &matrix_map, const std::vector<int> &indices)
{
    mean = std::accumulate(indices.cbegin(), indices.cend(), Eigen::Vector3d(0,0,0), [&](const Eigen::Vector3d &acc, int index)
    {
        return acc + matrix_map.col(index);
    }) / matrix_map.size();
}

//!
//! \brief calculate_weighted_mean
//!     A function that calculates the weighted mean of a data set. Running time is O(n).
//! \param mean
//!     The weighted mean (or centroid) of the data set
//! \param matrix_map
//!     A 3xN matrix map that correspondes to some point cloud. To get this type call pcl::PointCloud<PointT>::getMatrixXfMap.
//!     See: http://docs.pointclouds.org/trunk/classpcl_1_1_point_cloud.html#ab9486898ac363e8206829da7c01ea62d
//! \param vector_map
//!     A N dimensional vector that corresponds to the weight of some point
//!
template <class Derived1, class Derived2>
void calculate_weighted_mean(Eigen::Vector3d &mean, const Eigen::MatrixBase<Derived1> &matrix_map, const Eigen::MatrixBase<Derived2> &vector_map)
{
    mean.setZero(3);

    for (int i = 0; i < matrix_map.cols(); ++i)
    {
        mean += vector_map[i] * matrix_map.col(i);
    }
}

//!
//! \brief calculate_weighted_mean
//!     A function that calculates the weighted mean of a data set. Running time is O(n).
//! \param mean
//!     The weighted mean (or centroid) of the data set
//! \param matrix_map
//!     A 3xN matrix map that correspondes to some point cloud. To get this type call pcl::PointCloud<PointT>::getMatrixXfMap.
//!     See: http://docs.pointclouds.org/trunk/classpcl_1_1_point_cloud.html#ab9486898ac363e8206829da7c01ea62d
//! \param vector_map
//!     A N dimensional vector that corresponds to the weight of some point
//! \param indices
//!     The indices to use.
//!
template <class Derived1, class Derived2>
void calculate_weighted_mean(Eigen::Vector3d &mean, const Eigen::MatrixBase<Derived1> &matrix_map, const Eigen::MatrixBase<Derived2> &vector_map, const std::vector<int> &indices)
{
    mean = std::accumulate(indices.cbegin(), indices.cend(), Eigen::Vector3d(0,0,0), [&](const Eigen::Vector3d &acc, int index)
    {
        return acc + vector_map[index] * matrix_map.col(index);
    });
}

//!
//! \brief calculate_covariance
//!     A function that calculates the covariance matrix of a given point cloud. Running time is O(n).
//! \param covariance
//!     The covariance matrix of the point cloud
//! \param mean
//!     The mean value (or centroid) of the point cloud
//! \param matrix_map
//!     A 3xN matrix map that correspondes to some point cloud. To get this type call pcl::PointCloud<PointT>::getMatrixXfMap.
//!     See: http://docs.pointclouds.org/trunk/classpcl_1_1_point_cloud.html#ab9486898ac363e8206829da7c01ea62d
//! \param indices
//!     A list of indices to use.
//!
template <class Derived>
void calculate_covariance(Eigen::Matrix3d &covariance, const Eigen::Vector3d &mean, const Eigen::MatrixBase<Derived> &matrix_map)
{
    auto demeaned = matrix_map.transpose().rowwise() - mean.transpose();
    covariance = (demeaned.adjoint() * demeaned) / double(matrix_map.cols() - 1);
}

//!
//! \brief calculate_covariance
//!     A function that calculates the covariance of a given point cloud. Running time is O(n).
//! \param mean
//!     The mean value (or centroid) of the point cloud
//! \param covariance
//!     The covariance matrix of the point cloud
//! \param matrix_map
//!     A 3xN matrix map that correspondes to some point cloud. To get this type call pcl::PointCloud<PointT>::getMatrixXfMap.
//!     See: http://docs.pointclouds.org/trunk/classpcl_1_1_point_cloud.html#ab9486898ac363e8206829da7c01ea62d
//!
template <class Derived>
void calculate_covariance(Eigen::Matrix3d &covariance, const Eigen::Vector3d &mean, const Eigen::MatrixBase<Derived> &matrix_map, const std::vector<int> &indices)
{
    const size_t N = indices.size();

    covariance.setZero(3,3);

    for (const auto &index : indices)
    {
        auto u = matrix_map.col(index) - mean;

        covariance(0,0) += u[0] * u[0] / N;
        covariance(0,1) += u[0] * u[1] / N;
        covariance(0,2) += u[0] * u[2] / N;
        covariance(1,1) += u[1] * u[1] / N;
        covariance(1,2) += u[1] * u[2] / N;
        covariance(2,2) += u[1] * u[2] / N;
    }

    covariance(1,0) = covariance(0,1);
    covariance(2,0) = covariance(0,2);
    covariance(2,1) = covariance(1,2);
}

//!
//! \brief calculate_weighted_covariance
//!     A function that calculates the weighted covariance of a given point cloud. Running time is O(n).
//! \param covariance
//!     The covariance matrix of the point cloud
//! \param mean
//!     The mean value (or centroid) of the point cloud
//! \param matrix_map
//!     A 3xN matrix map that correspondes to some point cloud. To get this type call pcl::PointCloud<PointT>::getMatrixXfMap.
//!     See: http://docs.pointclouds.org/trunk/classpcl_1_1_point_cloud.html#ab9486898ac363e8206829da7c01ea62d
//! \param vector_map
//!     A N dimensional vector that corresponds to the weight of some point
//! \param sum_squared_weights
//!     The one minus sum of squared weights.
//!     See: https://en.wikipedia.org/wiki/Sample_mean_and_covariance#Weighted_samples
//!
template <class Derived1, class Derived2>
void calculate_weighted_covariance(Eigen::Matrix3d &covariance, const Eigen::Vector3d &mean, const Eigen::MatrixBase<Derived1> &matrix_map,  const Eigen::MatrixBase<Derived2> &vector_map, double sum_squared_weights)
{
    covariance.setZero(3,3);

    for (int index = 0; index < matrix_map.cols(); ++index)
    {
        auto u = matrix_map.col(index) - mean;

        covariance(0,0) += vector_map[index] * u[0] * u[0] / sum_squared_weights;
        covariance(0,1) += vector_map[index] * u[0] * u[1] / sum_squared_weights;
        covariance(0,2) += vector_map[index] * u[0] * u[2] / sum_squared_weights;
        covariance(1,1) += vector_map[index] * u[1] * u[1] / sum_squared_weights;
        covariance(1,2) += vector_map[index] * u[1] * u[2] / sum_squared_weights;
        covariance(2,2) += vector_map[index] * u[1] * u[2] / sum_squared_weights;
    }

    covariance(1,0) = covariance(0,1);
    covariance(2,0) = covariance(0,2);
    covariance(2,1) = covariance(1,2);
}

//!
//! \brief calculate_weighted_covariance
//!     A function that calculates the weighted covariance of a given point cloud. Running time is O(n).
//! \param covariance
//!     The covariance matrix of the point cloud
//! \param mean
//!     The mean value (or centroid) of the point cloud
//! \param matrix_map
//!     A 3xN matrix map that correspondes to some point cloud. To get this type call pcl::PointCloud<PointT>::getMatrixXfMap.
//!     See: http://docs.pointclouds.org/trunk/classpcl_1_1_point_cloud.html#ab9486898ac363e8206829da7c01ea62d
//! \param vector_map
//!     A N dimensional vector that corresponds to the weight of some point
//! \param sum_squared_weights
//!     The one minus sum of squared weights.
//!     See: https://en.wikipedia.org/wiki/Sample_mean_and_covariance#Weighted_samples
//! \param indices
//!     A list of indices to use.
//!
template <class Derived1, class Derived2>
void calculate_weighted_covariance(Eigen::Matrix3d &covariance, const Eigen::Vector3d &mean, const Eigen::MatrixBase<Derived1> &matrix_map,  const Eigen::MatrixBase<Derived2> &vector_map, double sum_squared_weights, const std::vector<int> &indices)
{
    covariance.setZero(3,3);

    for (const auto &index : indices)
    {
        auto u = matrix_map.col(index) - mean;

        covariance(0,0) += vector_map[index] * u[0] * u[0] / sum_squared_weights;
        covariance(0,1) += vector_map[index] * u[0] * u[1] / sum_squared_weights;
        covariance(0,2) += vector_map[index] * u[0] * u[2] / sum_squared_weights;
        covariance(1,1) += vector_map[index] * u[1] * u[1] / sum_squared_weights;
        covariance(1,2) += vector_map[index] * u[1] * u[2] / sum_squared_weights;
        covariance(2,2) += vector_map[index] * u[1] * u[2] / sum_squared_weights;
    }

    covariance(1,0) = covariance(0,1);
    covariance(2,0) = covariance(0,2);
    covariance(2,1) = covariance(1,2);
}

//!
//! \brief calculate_mean_and_covariance
//!     A function that calculates the mean value (or centroid) and the covariance of a given point cloud. Running time is O(n).
//! \param mean
//!     The mean value (or centroid) of the point cloud
//! \param covariance
//!     The covariance matrix of the point cloud
//! \param matrix_map
//!     A 3xN matrix map that correspondes to some point cloud. To get this type call pcl::PointCloud<PointT>::getMatrixXfMap.
//!     See: http://docs.pointclouds.org/trunk/classpcl_1_1_point_cloud.html#ab9486898ac363e8206829da7c01ea62d
//!
template <class Derived>
void calculate_mean_and_covariance(Eigen::Vector3d &mean, Eigen::Matrix3d &covariance, const Eigen::MatrixBase<Derived> &matrix_map)
{
    calculate_mean(mean, matrix_map);
    calculate_covariance(covariance, mean, matrix_map);
}

//!
//! \brief calculate_mean_and_covariance
//!     A function that calculates the mean value (or centroid) and the covariance of a given point cloud. Running time is O(n).
//! \param mean
//!     The mean value (or centroid) of the point cloud
//! \param covariance
//!     The covariance matrix of the point cloud
//! \param matrix_map
//!     A 3xN matrix map that correspondes to some point cloud. To get this type call pcl::PointCloud<PointT>::getMatrixXfMap.
//!     See: http://docs.pointclouds.org/trunk/classpcl_1_1_point_cloud.html#ab9486898ac363e8206829da7c01ea62d
//! \param indices
//!     A list of indices to use.
//!
template <class Derived>
void calculate_mean_and_covariance(Eigen::Vector3d &mean, Eigen::Matrix3d &covariance, const Eigen::MatrixBase<Derived> &matrix_map, const std::vector<int> &indices)
{
    calculate_mean(mean, matrix_map, indices);
    calculate_covariance(covariance, mean, matrix_map, indices);
}

//!
//! \brief calculate_weighted_mean_and_covariance
//!     A functionat that calculates the weighted mean (or centroid) and the weighted covariance of a point cloud
//! \param mean
//!     The mean value (or centroid) of the point cloud
//! \param covariance
//!     The covariance matrix of the point cloud
//! \param matrix_map
//!     A 3xN matrix map that correspondes to some point cloud. To get this type call pcl::PointCloud<PointT>::getMatrixXfMap.
//!     See: http://docs.pointclouds.org/trunk/classpcl_1_1_point_cloud.html#ab9486898ac363e8206829da7c01ea62d
//! \param vector_map
//!     A N dimensional vector that corresponds to the weight of some point
//! \param sum_squared_weights
//!     The one minus sum of squared weights.
//!     See: https://en.wikipedia.org/wiki/Sample_mean_and_covariance#Weighted_samples
//!
template <class Derived1, class Derived2>
void calculate_weighted_mean_and_covariance(Eigen::Vector3d &mean, Eigen::Matrix3d &covariance, const Eigen::MatrixBase<Derived1> &matrix_map, const Eigen::MatrixBase<Derived2> &vector_map, double sum_squared_weights)
{
    calculate_weighted_mean(mean, matrix_map, vector_map);
    calculate_weighted_covariance(covariance, mean, matrix_map, vector_map, sum_squared_weights);
}

//!
//! \brief calculate_weighted_mean_and_covariance
//!     A functionat that calculates the weighted mean (or centroid) and the weighted covariance of a point cloud
//! \param mean
//!     The mean value (or centroid) of the point cloud
//! \param covariance
//!     The covariance matrix of the point cloud
//! \param matrix_map
//!     A 3xN matrix map that correspondes to some point cloud. To get this type call pcl::PointCloud<PointT>::getMatrixXfMap.
//!     See: http://docs.pointclouds.org/trunk/classpcl_1_1_point_cloud.html#ab9486898ac363e8206829da7c01ea62d
//! \param vector_map
//!     A N dimensional vector that corresponds to the weight of some point
//! \param sum_squared_weights
//!     The one minus sum of squared weights.
//!     See: https://en.wikipedia.org/wiki/Sample_mean_and_covariance#Weighted_samples
//! \param indices
//!     A list of indices to use.
//!
template <class Derived1, class Derived2>
void calculate_weighted_mean_and_covariance(Eigen::Vector3d &mean, Eigen::Matrix3d &covariance, const Eigen::MatrixBase<Derived1> &matrix_map, const Eigen::MatrixBase<Derived2> &vector_map, double sum_squared_weights, const std::vector<int> &indices)
{
    calculate_weighted_mean(mean, matrix_map, vector_map, indices);
    calculate_weighted_covariance(covariance, mean, matrix_map, vector_map, sum_squared_weights, indices);
}

////!
////! \brief calculate_entropy
////!     A function that calculates the entropy of a binary classifier. Running time is O(1).
////!     See: https://en.wikipedia.org/wiki/Entropy_(information_theory)
////! \param entropy
////!     The entropy (or gain) of a decision
////! \param p
////!     The probability of a decision
////!
void ALIGNMENTPREDICTIONLIBSHARED_EXPORT calculate_entropy(double &entropy, double p);

#undef _USE_MATH_DEFINES

#endif // MULTIVARIATE_STATISTICS_H
