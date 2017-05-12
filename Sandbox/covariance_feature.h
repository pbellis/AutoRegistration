#ifndef COVARIANCE_FEATURE_H
#define COVARIANCE_FEATURE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Dense>
#include <vector>

#include "mean.h"
#include "covariance.h"
#include "statistical_distance.h"

template <class PointT>
void calculate_point_correlation(std::vector<Eigen::MatrixXd> &point_correlations, const pcl::PointCloud<PointT> &cloud, const pcl::KdTreeFLANN<PointT> &kdtree, int k)
{
    const size_t n = cloud.size();
    point_correlations.clear();
    point_correlations.reserve(n);

    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;

    Eigen::VectorXd mean;
    Eigen::MatrixXd covariance;
    Eigen::MatrixXd correlation;


    auto matrix_map = cloud.getMatrixXfMap().cast<double>().transpose();

    for (const auto &point : cloud.points)
    {
        kdtree.nearestKSearch(point, k, k_indices, k_sqr_distances);
        calculate_mean(mean, matrix_map, k_indices);
        calculate_covariance(covariance, matrix_map.rowwise() - mean.transpose(), k_indices);
        calculate_correlation_coefficient(correlation, covariance);
        point_correlations.push_back(correlation);
    }
}

void calculate_covariance_feature(std::vector<double> &features, const std::vector<Eigen::MatrixXd> &point_correlations, const Eigen::MatrixXd &correlation)
{
    const size_t n = point_correlations.size();

    features.reserve(n);
    features.clear();

    for (const auto &point_correlation : point_correlations)
    {
        features.push_back(0);
        calculate_multivariate_hellinger_distance(features.back(), point_correlation.block<3,3>(0,0), correlation.block<3,3>(0,0));
    }
}

void feature_color(pcl::PointCloud<pcl::PointXYZRGB>& rgb_cloud, const std::vector<double> &features, const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    const size_t n = cloud.size();
    rgb_cloud.reserve(n);
    rgb_cloud.clear();

    auto result = std::minmax_element(features.cbegin(), features.cend());
    double min_feature = 0; //*result.first;
    double max_feature = 1; //*result.second;

    for (int i = 0; i < n; ++i)
    {
        double u = (features[i] - min_feature) / max_feature;

        pcl::PointXYZRGB point;
        point.x = cloud.points[i].x;
        point.y = cloud.points[i].y;
        point.z = cloud.points[i].z;
        point.b = static_cast<char>(u * 255);
        point.r = 255 - point.b;
        point.g = (point.b / 2) + (point.r / 2);
        rgb_cloud.push_back(point);
    }
}

#endif // COVARIANCE_FEATURE_H
