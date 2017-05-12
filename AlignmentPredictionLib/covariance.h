#ifndef COVARIANCE_H
#define COVARIANCE_H

#include <Eigen/Dense>
#include "numeric_stability.h"

// TODO:
// Update documentation

//!
//! \brief calculate_covariance
//!     A function that calculates the covariance matrix of a given pointcloud.
//!     This uses kahan_summation to increase numerical stability.
//! \param covariance
//!     A d by d matrix that represents the covariance matrix of some point cloud.
//! \param matrix_map
//!     A d by n matrix map that correspondes to some point cloud.
//! \param mean
//!     A column vector which describes the mean of some point cloud.
//!
template <class Derived>
void calculate_covariance(Eigen::MatrixXd &covariance, const Eigen::MatrixBase<Derived> &matrix_map, const Eigen::VectorXd &mean)
{
    const size_t d = matrix_map.rows();
    const size_t n = matrix_map.cols();

    covariance.resize(d, d);

    std::vector<double> terms(n);
    for (int i = 0; i < d; ++i)
    {
        for (int j = i; j < d; ++j)
        {
            for (int index = 0; index < n; ++index)
            {
                auto point = matrix_map.col(index) - mean;
                terms[index] = point[i] * (point[j] / n);
            }

            covariance(i, j) = 0;
            kahan_summation(covariance(i, j), terms.begin(), terms.end());
            covariance(j, i) = covariance(i, j);
        }
    }
}

//!
//! \brief calculate_covariance
//!     A function that calculates the covariance matrix of a given pointcloud.
//!     This uses kahan_summation to increase numerical stability.
//! \param covariance
//!     A d by d matrix that represents the covariance matrix of some point cloud.
//! \param matrix_map
//!     A n by d matrix map that correspondes to some point cloud.
//! \param mean
//!     A column vector which describes the mean of some point cloud.
//! \param indices
//!     A vector containing all indices that should be used in covariance calculation.
//!
template <class Derived>
void calculate_covariance(Eigen::MatrixXd &covariance, const Eigen::MatrixBase<Derived> &matrix_map, const Eigen::VectorXd &mean, const std::vector<int> &indices)
{
    const size_t d = matrix_map.rows();
    const size_t n = indices.size();

    covariance.resize(d, d);

    std::vector<double> terms(n);
    for (int i = 0; i < d; ++i)
    {
        for (int j = i; j < d; ++j)
        {
            for (int index = 0; index < n; ++index)
            {
                auto point = matrix_map.col(indices[index]) - mean;
                terms[index] = point[i] * (point[j] / n);
            }

            covariance(i, j) = 0;
            kahan_summation(covariance(i, j), terms.begin(), terms.end());
            covariance(j, i) = covariance(i, j);
        }
    }
}

//!
//! \brief calculate_covariance
//!     A function that calculates the covariance matrix of a given pointcloud.
//!     This uses kahan_summation to increase numerical stability.
//! \param covariance
//!     A d by d matrix that represents the covariance matrix of some point cloud.
//! \param matrix_map
//!     A n by d matrix map that correspondes to some point cloud.
//! \param mean
//!     A column vector which describes the mean of some point cloud.
//! \param weights
//!     A vector of how important a point is in calculating covariance
//!
template <class Derived, class Derived2>
void calculate_covariance(Eigen::MatrixXd &covariance, const Eigen::MatrixBase<Derived> &matrix_map, const Eigen::VectorXd &mean, const  Eigen::MatrixBase<Derived2> &weights)
{
    const size_t d = matrix_map.rows();
    const size_t n = matrix_map.cols();

    covariance.resize(d, d);

    std::vector<double> terms(n);
    for (int i = 0; i < d; ++i)
    {
        for (int j = i; j < d; ++j)
        {
            for (int index = 0; index < n; ++index)
            {
                auto point = matrix_map.col(index) - mean;
                terms[index] = point[i] * weights[index] * point[j];
            }

            covariance(i, j) = 0;
            kahan_summation(covariance(i, j), terms.begin(), terms.end());
            covariance(j, i) = covariance(i, j);
        }
    }
}

//!
//! \brief calculate_correlation_coefficient
//!     A function that calculates the correlation coefficient given
//!     a covariance matrix
//! \param correlation_coefficient
//!     The correlation coefficient matrix calculated from the covariance
//!     matrix
//! \param covariance
//!     The covariance matrix used to calculate the correlation coefficient
//!     matrix. Look here: https://en.wikipedia.org/wiki/Correlation_coefficient
//!
//void calculate_correlation_coefficient(Eigen::MatrixXd &correlation_coefficient, const Eigen::MatrixXd &covariance)
//{
//    size_t d = covariance.rows();
//    correlation_coefficient.resize(d, d);

//    Eigen::VectorXd sigma(d);
//    for (int i = 0; i < d; ++i)
//    {
//        sigma[i] = std::sqrt(covariance(i, i));
//    }

//    for (int i = 0; i < d; ++i)
//    {
//        for (int j = i; j < d; ++j)
//        {
//            correlation_coefficient(i, j) = correlation_coefficient(j, i) = covariance(i, j) / (sigma[i] * sigma[j]);
//        }
//    }
//}

#endif // COVARIANCE_H
