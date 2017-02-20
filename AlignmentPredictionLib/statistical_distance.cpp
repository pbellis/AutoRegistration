#include "statistical_distance.h"

void calculate_multivariate_hellinger_distance(double &distance, const Eigen::Vector3d &source_mean, const Eigen::Matrix3d &source_covariance, const Eigen::Vector3d &target_mean, const Eigen::Matrix3d &target_covariance)
{
    const double scd = source_covariance.determinant();
    const double tcd = target_covariance.determinant();
    const Eigen::Matrix3d mc = 0.5 * (source_covariance + target_covariance);
    const double mcd = mc.determinant();
    const Eigen::Vector3d md = source_mean - target_mean;
    distance =  1 - std::exp(-0.125 * md.transpose() * mc.inverse() * md + 0.25 * std::log(scd * tcd / (mcd * mcd)));
}

void calculate_multivariate_hellinger_distance(Eigen::MatrixXd &distance_matrix, const std::vector<Eigen::Vector3d> &means, const std::vector<Eigen::Matrix3d> &covariances)
{
    const size_t N = means.size();

    for (size_t i = 0; i < N; ++i)
    {
        distance_matrix(i,  i) = 0;
        for (size_t j = i + 1; j < N; ++j)
        {
            calculate_multivariate_hellinger_distance(distance_matrix(i, j), means[i], covariances[i], means[j], covariances[j]);
            distance_matrix(j, i) = distance_matrix(i, j);
        }
    }
}
