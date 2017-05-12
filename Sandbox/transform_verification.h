#ifndef TRANSFORM_VERIFICATION_H
#define TRANSFORM_VERIFICATION_H

#include <pcl/kdtree/kdtree_flann.h>

struct Correspondences
{
    std::vector<int> source_indices;
    std::vector<int> target_indices;
};

template <class PointT>
Correspondences calculate_correspondences(typename pcl::PointCloud<PointT>::Ptr source_cloud, typename pcl::PointCloud<PointT>::Ptr target_cloud, float max_distance)
{
    Correspondences m;

    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr source_kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr target_kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>);

    std::vector<int> source_k_indices (1);
    std::vector<int> target_k_indices (1);
    std::vector<float> k_dists (1);
    source_kdtree->setInputCloud(source_cloud);
    target_kdtree->setInputCloud(target_cloud);
    for (int i = 0; i < target_cloud->size(); ++i)
    {
        source_kdtree->nearestKSearch(target_cloud->points[i], 1, source_k_indices, k_dists);
        if (k_dists[0] < max_distance)
        {
            target_kdtree->nearestKSearch(source_cloud->points[source_k_indices[0]], 1, target_k_indices, k_dists);
            if (target_k_indices[0] == i)
            {
                m.source_indices.push_back(source_k_indices[0]);
                m.target_indices.push_back(i);
            }
        }
    }
    return m;
}

template <class PointT>
float calculate_error(typename pcl::PointCloud<PointT>::Ptr source_cloud, typename pcl::PointCloud<PointT>::Ptr target_cloud, const Correspondences& m)
{
    float error = 0.0;
    for (int i = 0; i < m.target_indices.size(); ++i)
    {
        error += pcl::squaredEuclideanDistance<pcl::PointXYZ, pcl::PointXYZ>(source_cloud->points[m.source_indices[i]], target_cloud->points[m.target_indices[i]]);
    }
    return error;
}

#endif // TRANSFORM_VERIFICATION_H
