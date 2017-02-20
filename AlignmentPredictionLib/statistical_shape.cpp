#include "statistical_shape.h"

#include "multivariate_statistics.h"

#include <algorithm>
#include <numeric>
#include <unsupported/Eigen/MatrixFunctions>

void calculate_ownership_uncertainty(std::vector<double> &source_ownership_uncertainty, const Eigen::Vector3d &source_mean, const Eigen::Matrix3d &source_covariance, const pcl::PointCloud<pcl::PointXYZ> &source_cloud, const Eigen::Vector3d &target_mean, const Eigen::Matrix3d &target_covariance, size_t target_size)
{
        const size_t source_size = source_cloud.size();

        const double pt = static_cast<double>(target_size) / static_cast<double>(source_size + target_size);

        const Eigen::Matrix3d source_precision = source_covariance.inverse();
        const Eigen::Matrix3d target_precision = target_covariance.inverse();

        const double ts = std::pow(2 * M_PI, -1.5) * std::pow(source_covariance.determinant(), -0.5);
        const double tt = std::pow(2 * M_PI, -1.5) * std::pow(target_covariance.determinant(), -0.5);

        source_ownership_uncertainty.clear();
        source_ownership_uncertainty.reserve(source_size);

        auto matrix_map = source_cloud.getMatrixXfMap(3, 4, 0).cast<double>();

        for (int i = 0; i < source_size; ++i)
        {
            const Eigen::Vector3d dms = matrix_map.col(i) - source_mean;
            const Eigen::Vector3d dmt = matrix_map.col(i) - target_mean;

            const double pxs = ts * std::exp(-0.5 * dms.transpose() * source_precision * dms);
            const double pxt = tt * std::exp(-0.5 * dmt.transpose() * target_precision * dmt);

            const double p = pxt * pt / (pxs + pxt);
            double entropy;

            calculate_entropy(entropy, p);

            source_ownership_uncertainty.push_back(entropy);
        }
}

void calculate_ownership_uncertainty(std::vector<double> &source_ownership_uncertainty, const Eigen::Vector3d &source_mean, const Eigen::Matrix3d &source_covariance, const pcl::PointCloud<pcl::PointXYZ> &source_cloud, const std::vector<int> &source_indices, const Eigen::Vector3d &target_mean, const Eigen::Matrix3d &target_covariance, size_t target_size)
{
    const size_t source_size = source_cloud.size();

    const double pt = static_cast<double>(target_size) / static_cast<double>(source_size + target_size);

    const Eigen::Matrix3d source_precision = source_covariance.inverse();
    const Eigen::Matrix3d target_precision = target_covariance.inverse();

    const double ts = std::pow(2 * M_PI, -1.5) * std::pow(source_covariance.determinant(), -0.5);
    const double tt = std::pow(2 * M_PI, -1.5) * std::pow(target_covariance.determinant(), -0.5);

    source_ownership_uncertainty.clear();
    source_ownership_uncertainty.resize(source_size);

    auto matrix_map = source_cloud.getMatrixXfMap(3, 4, 0).cast<double>();

    for (const auto &index : source_indices)
    {
        const Eigen::Vector3d dms = matrix_map.col(index) - source_mean;
        const Eigen::Vector3d dmt = matrix_map.col(index) - target_mean;

        const double pxs = ts * std::exp(-0.5 * dms.transpose() * source_precision * dms);
        const double pxt = tt * std::exp(-0.5 * dmt.transpose() * target_precision * dmt);

        const double p = pxt * pt / (pxs + pxt);
        calculate_entropy(source_ownership_uncertainty[index], p);
    }
}

void find_uncertain_region(std::vector<int> &uncertain_indices, const std::vector<double> &ownership_uncertainty, const pcl::PointCloud<pcl::PointXYZ> &cloud, double total_uncertainty_probability)
{
    const size_t cloud_size = cloud.size();

    auto matrix_map = cloud.getMatrixXfMap(3, 4, 0).cast<double>();
    auto vector_map = Eigen::VectorXd::Map(ownership_uncertainty.data(), cloud_size);
    const double total = vector_map.sum();
    auto normalized_vector_map = (1 / total) * vector_map;
    auto inverse_vector_map = (1 - vector_map.array()).matrix();
    auto inverse_normalized_vector_map = (1 / (static_cast<double>(cloud_size) - total)) * inverse_vector_map;
    const double sum_squared_weights = 1 - normalized_vector_map.dot(normalized_vector_map);
    const double inverse_sum_squared_weights = 1 - inverse_normalized_vector_map.dot(inverse_normalized_vector_map);

    Eigen::Vector3d uncertain_mean;
    Eigen::Matrix3d uncertain_covariance;
    calculate_weighted_mean_and_covariance(uncertain_mean, uncertain_covariance, matrix_map, normalized_vector_map, sum_squared_weights);

    Eigen::Vector3d certain_mean;
    Eigen::Matrix3d certain_covariance;
    calculate_weighted_mean_and_covariance(certain_mean, certain_covariance, matrix_map, inverse_normalized_vector_map, inverse_sum_squared_weights);

    const Eigen::Matrix3d uncertain_precision = uncertain_covariance.inverse();
    const Eigen::Matrix3d certain_precision = certain_covariance.inverse();

    const double uncertain_term = std::pow(2 * M_PI, -1.5) * std::pow(uncertain_covariance.determinant(), -0.5);
    const double certain_term = std::pow(2 * M_PI, -1.5) * std::pow(certain_covariance.determinant(), -0.5);

    for (int i = 0; i < cloud_size; ++i)
    {
        const Eigen::Vector3d uncertain_demeaned = matrix_map.col(i) - uncertain_mean;
        const double uncertain_probability = uncertain_term * std::exp(-0.5 * uncertain_demeaned.transpose() * uncertain_precision * uncertain_demeaned);

        const Eigen::Vector3d certain_demeaned = matrix_map.col(i) - certain_mean;
        const double certain_probability = certain_term * std::exp(-0.5 * certain_demeaned.transpose() * certain_precision * certain_demeaned);

        if (uncertain_probability > certain_probability)
        {
            uncertain_indices.push_back(i);
        }
    }
}

void calcuate_statistical_shape(std::vector<StatisticalShape> &statistical_shape, const pcl::PointCloud<pcl::PointXYZ> &cloud, const pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree, double radius)
{
    const size_t N = cloud.size();
    statistical_shape.clear();
    statistical_shape.reserve(N);

    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;

    Eigen::Vector3d mean;
    Eigen::Matrix3d covariance;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver;
    Eigen::Vector3d eigen_values;

    auto matrix_map = cloud.getMatrixXfMap(3, 4, 0).cast<double>();

    for (const auto &point : cloud.points)
    {
        kdtree.radiusSearch(point, radius, k_indices, k_sqr_distances);
        calculate_mean_and_covariance(mean, covariance, matrix_map, k_indices);
        eigen_values = eigen_solver.computeDirect(covariance, Eigen::EigenvaluesOnly).eigenvalues();
        statistical_shape.push_back(
                    StatisticalShape{
                        (eigen_values[2] - eigen_values[1]) / eigen_values[2],
                        (eigen_values[1] - eigen_values[0]) / eigen_values[2],
                        (eigen_values[0] / eigen_values[2]),
                        (eigen_values[0] * eigen_values[1] * eigen_values[2]),
                        (eigen_values[2] - eigen_values[0]) / eigen_values[2],
                        eigen_values[0] / (eigen_values[0] + eigen_values[1] + eigen_values[2])
                    });
    }
}

void calcuate_statistical_shape(std::vector<StatisticalShape> &statistical_shape, const pcl::PointCloud<pcl::PointXYZ> &cloud, const pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree, double radius, const std::vector<int> &indices)
{
    const size_t N = cloud.size();
    statistical_shape.clear();
    statistical_shape.resize(N);

    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;

    Eigen::Vector3d mean;
    Eigen::Matrix3d covariance;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver;
    Eigen::Vector3d eigen_values;

    auto matrix_map = cloud.getMatrixXfMap(3, 4, 0).cast<double>();

    for (const auto &index : indices)
    {
        const auto &point = cloud.points[index];

        kdtree.radiusSearch(point, radius, k_indices, k_sqr_distances);
        calculate_mean_and_covariance(mean, covariance, matrix_map, k_indices);
        eigen_values = eigen_solver.computeDirect(covariance, Eigen::EigenvaluesOnly).eigenvalues();
        statistical_shape[index] =
                StatisticalShape{
                        (eigen_values[2] - eigen_values[1]) / eigen_values[2],
                        (eigen_values[1] - eigen_values[0]) / eigen_values[2],
                        (eigen_values[0] / eigen_values[2]),
                        (eigen_values[0] * eigen_values[1] * eigen_values[2]),
                        (eigen_values[2] - eigen_values[0]) / eigen_values[2],
                        eigen_values[0] / (eigen_values[0] + eigen_values[1] + eigen_values[2])
                    };
    }
}
