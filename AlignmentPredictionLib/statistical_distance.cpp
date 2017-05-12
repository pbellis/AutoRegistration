#include "statistical_distance.h"

void calculate_multivariate_hellinger_distance(double &distance, const Eigen::MatrixXd &source_covariance, const Eigen::MatrixXd &target_covariance)
{
    const double scd = source_covariance.determinant();
    const double tcd = target_covariance.determinant();
    const Eigen::MatrixXd mc = 0.5 * (source_covariance + target_covariance);
    const double mcd = mc.determinant();
    distance = std::pow((scd * tcd) / (mcd * mcd), 0.25);
}

void calculate_multivariate_hellinger_distance(double &distance, const Eigen::VectorXd &source_mean, const Eigen::MatrixXd &source_covariance, const Eigen::VectorXd &target_mean, const Eigen::MatrixXd &target_covariance)
{
    const double scd = source_covariance.determinant();
    const double tcd = target_covariance.determinant();
    const Eigen::MatrixXd mc = 0.5 * (source_covariance + target_covariance);
    const double mcd = mc.determinant();
    const Eigen::VectorXd md = source_mean - target_mean;
    distance =  1 - std::exp(-0.125 * md.transpose() * mc.inverse() * md + 0.25 * std::log(scd * tcd / (mcd * mcd)));
}

void calculate_multivariate_hellinger_distance(Eigen::MatrixXd &distance_matrix, const std::vector<Eigen::VectorXd> &means, const std::vector<Eigen::MatrixXd> &covariances)
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

void calculate_mahalanobis_distance(double &distance, const Eigen::VectorXd &point, const Eigen::VectorXd &mean, const Eigen::MatrixXd &precision)
{
    auto demeaned = point - mean;
    distance = sqrt(demeaned.transpose() * precision * demeaned);
}

void calculate_multivariate_pdf(double &pdf, const Eigen::VectorXd &point, double det, const Eigen::VectorXd &mean, const Eigen::MatrixXd &precision)
{
    double pi_term = 1.0 / std::pow(2 * M_PI, point.cols() / 2);
    double det_term = 1.0 / std::sqrt(det);
    auto u = point - mean;
    double e_term = 1.0 / std::exp(0.5 * u.transpose() * precision * u);
    pdf = pi_term * det_term * e_term;
}
