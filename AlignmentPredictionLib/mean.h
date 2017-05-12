#ifndef MEAN_H
#define MEAN_H

#include <Eigen/Dense>
#include <vector>
#include <numeric>

#include "numeric_stability.h"

//!
//! \brief calculate_mean
//!     A function that calculates the mean value (or centroid) of a given point cloud. Running time is O(n).
//! \param mean
//!     The mean value (or centroid) of the point cloud
//! \param matrix_map
//!     A n by 3 matrix map that correspondes to some point cloud. To get this type call pcl::PointCloud<PointT>::getMatrixXfMap.
//!     See: http://docs.pointclouds.org/trunk/classpcl_1_1_point_cloud.html#ab9486898ac363e8206829da7c01ea62d
//!
template <class Derived>
void calculate_mean(Eigen::VectorXd &mean, const Eigen::MatrixBase<Derived> &matrix_map)
{
    mean = matrix_map.rowwise().mean();
}

//!
//! \brief calculate_mean
//!     A function that calculates the mean value (or centroid) of a given point cloud. Running time is O(n).
//! \param mean
//!     The mean value (or centroid) of the point cloud
//! \param matrix_map
//!     A d by n matrix map that correspondes to some point cloud. To get this type call pcl::PointCloud<PointT>::getMatrixXfMap.
//!     See: http://docs.pointclouds.org/trunk/classpcl_1_1_point_cloud.html#ab9486898ac363e8206829da7c01ea62d
//! \param indices
//!     A vector containing all indices that should be used in mean calculation
//!
template <class Derived>
void calculate_mean(Eigen::VectorXd &mean, const Eigen::MatrixBase<Derived> &matrix_map, const std::vector<int>& indices)
{
    const size_t n = indices.size();
    const size_t d = matrix_map.rows();
    mean = Eigen::VectorXd::Zero(d);

    std::vector<double> values;
    values.resize(n);

    for (auto i = 0; i < d; ++i)
    {
        std::transform(indices.cbegin(), indices.cend(), values.begin(), [&](int index)
        {
            return matrix_map(i, index) / n;
        });
        mean[i] = 0;
        kahan_summation(mean[i], values.cbegin(), values.cend());
    }
}

//!
//! \brief calculate_mean
//!     A function that calculates the mean value (or centroid) of a given point cloud. Running time is O(n).
//! \param mean
//!     The mean value (or centroid) of the point cloud
//! \param matrix_map
//!     A 3xN matrix map that correspondes to some point cloud. To get this type call pcl::PointCloud<PointT>::getMatrixXfMap.
//!     See: http://docs.pointclouds.org/trunk/classpcl_1_1_point_cloud.html#ab9486898ac363e8206829da7c01ea62d
//! \param weights
//!     A vector containing all weights that should be used in mean calculation
//!
template <class Derived, class Derived2>
void calculate_mean(Eigen::VectorXd &mean, const Eigen::MatrixBase<Derived> &matrix_map, const Eigen::MatrixBase<Derived2>& weights)
{
    mean = (weights.asDiagonal() * matrix_map).rowwise().sum();
}

#endif // MEAN_H
