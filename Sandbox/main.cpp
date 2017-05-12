#include <QString>
#include <QDirIterator>
#include <QFile>
#include <QTextStream>

#include <iostream>
#include <chrono>
#include <random>
#include <QMap>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/transformation_validation_euclidean.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>
#include <pcl/common/distances.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include "mean.h"
#include "covariance.h"
#include "statistical_distance.h"
#include "statistical_shape.h"
#include "graph_theory.h"
#include "eigen_utils.h";
#include "covariance_feature.h"
#include "input_handler.h"
#include "transform_verification.h"
#include "registration.h"

struct MultivariateGaussian
{
    Eigen::VectorXd mean;
    Eigen::MatrixXd covariance;
};

//int main(int argc, char *argv[])
//{
//    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

//    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vec = load_ply_files(QString(argv[1]), QString(argv[2]));

//    const size_t n = cloud_vec.size();

//    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//    std::vector<int> k_indices;
//    std::vector<float> k_sqr_distances;
//    double resolution = std::accumulate(cloud_vec.cbegin(), cloud_vec.cend(), 0.0, [&](double acc, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
//    {
//        kdtree.setInputCloud(cloud);

//        double local_resolution = std::accumulate(cloud->points.cbegin(), cloud->points.cend(), 0.0, [&](double acc, const pcl::PointXYZ &point)
//        {
//            kdtree.nearestKSearch(point, 2, k_indices, k_sqr_distances);
//            return acc + (k_sqr_distances[1] / cloud->size());
//        });
//        return acc + (local_resolution / cloud_vec.size());
//    });
//    resolution = sqrt(resolution);

//    const double radius = 20 * resolution;

//    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//    std::transform(cloud_vec.cbegin(), cloud_vec.cend(), cloud_vec.begin(), [&](const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
//    {
//        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//        sor.setInputCloud(cloud);
//        sor.setMeanK(50);
//        sor.setStddevMulThresh(1.0);
//        sor.filter(*filtered_cloud);
//        return filtered_cloud;
//    });

//    std::vector<Eigen::MatrixXd> statistical_shape_vec;
//    statistical_shape_vec.resize(n);
//    std::transform(cloud_vec.cbegin(), cloud_vec.cend(), statistical_shape_vec.begin(), [&](const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
//    {
//        kdtree.setInputCloud(cloud);
//        Eigen::MatrixXd statistical_shape;
//        calculate_statistical_shape(statistical_shape, *cloud, kdtree, 50);

//        return statistical_shape;
//    });

//    std::vector<Eigen::VectorXd> mean_vec;
//    mean_vec.resize(n);
//    std::transform(statistical_shape_vec.cbegin(), statistical_shape_vec.cend(), mean_vec.begin(), [](const Eigen::MatrixXd &statistical_shape)
//    {
//        Eigen::VectorXd mean;
//        calculate_mean(mean, statistical_shape);
//        return mean;
//    });

//    std::vector<Eigen::MatrixXd> covariance_vec;
//    covariance_vec.resize(n);
//    std::transform(statistical_shape_vec.cbegin(), statistical_shape_vec.cend(), mean_vec.cbegin(), covariance_vec.begin(), [](const Eigen::MatrixXd &statistical_shape, const Eigen::VectorXd &mean)
//    {
//        Eigen::MatrixXd covariance;
//        calculate_covariance(covariance, statistical_shape, mean);
//        return covariance;
//    });

//    std::vector<double> det_vec;
//    det_vec.resize(n);
//    std::transform(covariance_vec.cbegin(), covariance_vec.cend(), det_vec.begin(), [](const Eigen::MatrixXd &covariance)
//    {
//        return covariance.determinant();
//    });

//    std::vector<Eigen::MatrixXd> precision_vec;
//    precision_vec.resize(n);
//    std::transform(covariance_vec.cbegin(), covariance_vec.cend(), precision_vec.begin(), [](const Eigen::MatrixXd &covariance)
//    {
//        Eigen::MatrixXd precision = covariance.inverse();
//        return precision;
//    });

//    Eigen::VectorXd combined_mean = Eigen::VectorXd::Zero(3);
//    combined_mean = std::accumulate(mean_vec.cbegin(), mean_vec.cend(), combined_mean) / n;
//    Eigen::MatrixXd combined_covariance = Eigen::MatrixXd::Zero(3,3);
//    combined_covariance = std::accumulate(covariance_vec.cbegin(), covariance_vec.cend(), combined_covariance) / n;
//    Eigen::MatrixXd combined_precision = combined_covariance.inverse();
//    double combined_det = combined_covariance.determinant();

//    std::vector<Eigen::VectorXd> features_vec;
//    features_vec.resize(n);
//    for (int i = 0; i < n; ++i)
//    {
////        calculate_features(features_vec[i], statistical_shape_vec[i], det_vec[i], mean_vec[i], precision_vec[i]);
//        calculate_features(features_vec[i], statistical_shape_vec[i], combined_det, combined_mean, combined_precision);
//    }

//    std::vector<std::vector<int>> indices_vec;
//    indices_vec.resize(n);
//    std::transform(features_vec.cbegin(), features_vec.cend(), indices_vec.begin(), [](const Eigen::VectorXd &features)
//    {
//        Eigen::VectorXd mean;
//        calculate_mean(mean, features.transpose());

//        Eigen::MatrixXd sigma;
//        calculate_covariance(sigma, features.transpose(), mean);

//        double split = mean[0] + 1 * sigma(0,0);

//        std::vector<int> indices;
//        indices.reserve(features.rows() / 10);

//        for (auto i = 0; i < features.rows(); ++i)
//        {
//            if (features[i] > split)
//            {
//                indices.push_back(i);
//            }
//        }

//        return indices;
//    });

//    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> feature_cloud_vec;
//    feature_cloud_vec.resize(n);
//    std::transform(cloud_vec.cbegin(), cloud_vec.cend(), indices_vec.cbegin(), feature_cloud_vec.begin(), [](const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::vector<int> &indices)
//    {
//        pcl::PointCloud<pcl::PointXYZ>::Ptr feature_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::copyPointCloud(*cloud, indices, *feature_cloud);
//        return feature_cloud;
//    });


//    FeatureCloud source;
//    source.cloud = feature_cloud_vec[0];
//    source.features.reset(new pcl::PointCloud<pcl::PointXYZ>);
//    for (const auto & index : indices_vec[0])
//    {
//        Eigen::VectorXd col = statistical_shape_vec[0].col(index);
//        pcl::PointXYZ point;
//        point.x = col[0];
//        point.y = col[1];
//        point.z = col[2];
//        source.features->points.push_back(point);
//    }
//    source.cloud_kdtree.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
//    source.cloud_kdtree->setInputCloud(source.cloud);
//    source.features_kdtree.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
//    source.features_kdtree->setInputCloud(source.features);

//    FeatureCloud target;
//    target.cloud = feature_cloud_vec[1];
//    target.features.reset(new pcl::PointCloud<pcl::PointXYZ>);
//    for (const auto & index : indices_vec[1])
//    {
//        Eigen::VectorXd col = statistical_shape_vec[1].col(index);
//        pcl::PointXYZ point;
//        point.x = col[0];
//        point.y = col[1];
//        point.z = col[2];
//        target.features->points.push_back(point);
//    }
//    target.cloud_kdtree.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
//    target.cloud_kdtree->setInputCloud(target.cloud);
//    target.features_kdtree.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
//    target.features_kdtree->setInputCloud(target.features);

//    pcl::Correspondences correspondences;
//    calculate_reciprocal_correspondences(correspondences, source, target, 30, 30);

//    {
//        pcl::visualization::PCLVisualizer viewer;
//        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> source_rgb_handler(source.cloud);
//        viewer.addPointCloud(source.cloud, source_rgb_handler, "source_cloud");
//        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> target_rgb_handler(target.cloud);
//        viewer.addPointCloud(target.cloud, target_rgb_handler, "target_cloud");
//        viewer.addCorrespondences<pcl::PointXYZ>(source.cloud, target.cloud, correspondences);
//        viewer.spin();
//    }

//    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> rgb_cloud_vec;
//    rgb_cloud_vec.resize(n);
//    std::transform(cloud_vec.cbegin(), cloud_vec.cend(), features_vec.cbegin(), rgb_cloud_vec.begin(), [](const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const Eigen::VectorXd &features)
//    {
//        pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//        rgb_cloud->resize(cloud->size());
//        std::transform(cloud->points.cbegin(), cloud->points.cend(), features.data(), rgb_cloud->points.begin(), [](const pcl::PointXYZ &point, double feature)
//        {
//            pcl::PointXYZRGB rgb_point;
//            rgb_point.x = point.x;
//            rgb_point.y = point.y;
//            rgb_point.z = point.z;
//            rgb_point.r = feature * 255;
//            rgb_point.b = 255 - rgb_point.r;
//            rgb_point.g = rgb_point.r / 2 + rgb_point.b / 2;
//            return rgb_point;
//        });
//        return rgb_cloud;
//    });

//    std::cout << "displaying " << n << " clouds..." << std::endl;

//    for (int i = 0; i < n; ++i)
//    {
//        pcl::visualization::PCLVisualizer viewer;
//        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_handler(rgb_cloud_vec[i]);
//        viewer.addPointCloud(rgb_cloud_vec[i], rgb_handler, "rgb_cloud");
//        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> feature_handler(feature_cloud_vec[i]);
//        viewer.addPointCloud(feature_cloud_vec[i], feature_handler, "feature_cloud");
//        viewer.spin();
//    }
//}

int main(int argc, char *argv[])
{
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds = load_ply_files(QString(argv[1]), QString(argv[2]));

    const size_t n = clouds.size();
    const float theta = std::atof(argv[3]);

    double resolution;
    {
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(clouds[0]);

        std::vector<int> k_indices(2);
        std::vector<float> k_sqr_distances(2);

        std::for_each(clouds[0]->points.cbegin(), clouds[0]->points.cend(), [&](const pcl::PointXYZ &point)
        {
            kdtree.nearestKSearch(point, 2, k_indices, k_sqr_distances);
            resolution += sqrt(k_sqr_distances[1]) / clouds[0]->size();
        });
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> rough_clouds;
    rough_clouds.resize(n);
    std::transform(clouds.cbegin(), clouds.cend(), rough_clouds.begin(), [&](const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        Eigen::VectorXd mean;
        calculate_mean(mean, cloud->getMatrixXfMap().cast<double>());

        Eigen::Affine3f affine = Eigen::Translation3f(-mean.head<3>().cast<float>()) * random_angle_axis(-theta, theta) * Eigen::Translation3f(mean.head<3>().cast<float>());

        pcl::PointCloud<pcl::PointXYZ>::Ptr rough_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud, *rough_cloud, affine);

        return rough_cloud;
    });

    // CALCULATE MEAN AND COVARIANCES
    std::vector<MultivariateGaussian> dists(n);
    {
        std::cout << "calculating multivariate distributions... ";
        auto start = std::chrono::high_resolution_clock::now();
        std::transform(rough_clouds.cbegin(), rough_clouds.cend(), dists.begin(), [](const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
        {
            MultivariateGaussian dist;
            calculate_mean(dist.mean, cloud->getMatrixXfMap().cast<double>());
            calculate_covariance(dist.covariance, cloud->getMatrixXfMap().cast<double>(), dist.mean);
            return dist;
        });
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << "took " << std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count() << " secs" << std::endl;
    }

    std::vector<Eigen::VectorXd> means(n);
    std::transform(dists.cbegin(), dists.cend(), means.begin(), [](const MultivariateGaussian &dist)
    {
        return dist.mean.head<3>();
    });

    std::vector<Eigen::MatrixXd> covariances(n);
    std::transform(dists.cbegin(), dists.cend(), covariances.begin(), [](const MultivariateGaussian &dist)
    {
        return dist.covariance.block<3,3>(0,0);
    });

    Eigen::MatrixXd distance_matrix(n, n);
    {
        std::cout << "calculating distance matrix (hellinger)... ";
        auto start = std::chrono::high_resolution_clock::now();
        calculate_multivariate_hellinger_distance(distance_matrix, means, covariances);
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << "took " << std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count() << " secs" << std::endl;
        std::cout << "distance_matrix" << std::endl << distance_matrix << std::endl;
    }

    // CALCULATE THE MINIMUM SPANNING TREE OF THAT DISTANCE MATRIX
    double total_distance; Eigen::VectorXi minimum_spanning_tree(n);
    {
        std::cout << "calculating minimum spanning tree... ";
        auto start = std::chrono::high_resolution_clock::now();
        calculate_minimum_spanning_tree(total_distance, minimum_spanning_tree, distance_matrix);
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << "took " << std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count() << " secs" << std::endl;
        std::cout << "total_distance " << total_distance << std::endl;
        std::cout << "minimum_spanning_tree " << minimum_spanning_tree.transpose() << std::endl;
    }

    // CALCULATE THE DEGREE OF EACH SCAN
    Eigen::VectorXi node_degree(n);
    {
        std::cout << "calculating node degree... ";
        auto start = std::chrono::high_resolution_clock::now();
        calculate_degree(node_degree, minimum_spanning_tree);
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << "took " << std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count() << " secs" << std::endl;
        std::cout << "node_degree " << node_degree.transpose() << std::endl;
    }

    // CALCULATE THE BALANCE OF THE GRAPH
    Eigen::MatrixXd balance_matrix(n, n);
    {
        std::cout << "calculating balance matrix... ";
        auto start = std::chrono::high_resolution_clock::now();
        calculate_graph_balance(balance_matrix, node_degree, total_distance, minimum_spanning_tree, distance_matrix);
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << "took " << std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count() << " secs" << std::endl;
        std::cout << "balance_matrix " << balance_matrix.transpose() << std::endl;
    }

    // CALCULATE THE CENTER SCAN OF THE GRAPH (THIS IS WHAT WE WILL TRY TO ALIGN ALL SCANS TO)
    int center;
    {
        std::cout << "calculating balance matrix... ";
        auto start = std::chrono::high_resolution_clock::now();
        calculate_graph_center(center, balance_matrix, minimum_spanning_tree);
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << "took " << std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count() << " secs" << std::endl;
        std::cout << "center " << center << std::endl;
    }

    // CALCULATE WHICH SCANS SHOULD ALIGN TO WHICH SCANS
    Eigen::VectorXi path(n);
    {
        std::cout << "calculating optimal path... ";
        auto start = std::chrono::high_resolution_clock::now();
        calculate_optimal_path(path, center, minimum_spanning_tree, node_degree);
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << "took " << std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count() << " secs" << std::endl;
        std::cout << "path " << path.transpose() << std::endl;
    }

    Eigen::VectorXi distance_to_root(n);
    {
        std::cout << "calculating node distance...";
        auto start = std::chrono::high_resolution_clock::now();
        calculate_node_distances(distance_to_root, path, node_degree, center);
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << "took " << std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count() << " secs" << std::endl;
        std::cout << distance_to_root.transpose() << std::endl;
    }

    // SORT NODES BY DISTANCE TO CENTER
    Eigen::VectorXi sorted_nodes(n);
    {
        std::cout << "sorting nodes by distance to center... ";
        auto start = std::chrono::high_resolution_clock::now();
        sort_nodes_by_distance(sorted_nodes, distance_to_root);
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << "took " << std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count() << " secs" << std::endl;
        std::cout << "sorted nodes " << sorted_nodes.transpose() << std::endl;
    }

    // CALCULATE CORRESPONDENCES
    std::vector<Correspondences> correspondences;
    {
        std::cout << "calculating correspondnces... ";
        auto start = std::chrono::high_resolution_clock::now();

        for (int source = 0; source < n; ++source)
        {
            int target = path[source];
            correspondences.push_back(calculate_correspondences<pcl::PointXYZ>(clouds[source], clouds[target], 0.2f));
        }
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << "took " << std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count() << " secs" << std::endl;
    }

    // CALCULATE GROUND ERROR
    {
        float error = 0.0f;

        std::cout << "calculating ground error... ";
        auto start = std::chrono::high_resolution_clock::now();

        for (int source = 0; source < n; ++source)
        {
            int target = path[source];
            error += calculate_error<pcl::PointXYZ>(clouds[source], clouds[target], correspondences[source]);

        }

        auto end = std::chrono::high_resolution_clock::now();
        std::cout << "took " << std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count() << " secs" << std::endl;
        std::cout << "error: " << error << std::endl;
    }

    // VISUALIZE GROUND ALIGNMENT
    {
        pcl::visualization::PCLVisualizer viewer;
        for (int i = 0; i < n; ++i)
        {
            pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> handler(clouds[i]);
            viewer.addPointCloud(clouds[i], handler, std::to_string(i));
        }

        viewer.spin();
    }

    // CALCULATE INITIAL ERROR
    {
        float error = 0.0f;

        std::cout << "calculating initial error... ";
        auto start = std::chrono::high_resolution_clock::now();

        for (int source = 0; source < n; ++source)
        {
            int target = path[source];
            error += calculate_error<pcl::PointXYZ>(rough_clouds[source], rough_clouds[target], correspondences[source]);
        }

        auto end = std::chrono::high_resolution_clock::now();
        std::cout << "took " << std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count() << " secs" << std::endl;
        std::cout << "error: " << error << std::endl;
    }

    // VISUALIZE INITIAL ALIGNMENT
    {
        pcl::visualization::PCLVisualizer viewer;
        for (int i = 0; i < n; ++i)
        {
            pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> handler(rough_clouds[i]);
            viewer.addPointCloud(rough_clouds[i], handler, std::to_string(i));
        }
        viewer.spin();
    }

    // ROUGH ALIGNMENT
    {
        std::cout << "aligning (rough) point rough_clouds... ";
        auto start = std::chrono::high_resolution_clock::now();

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr icp(new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>);
        icp->setMaxCorrespondenceDistance (100 * resolution);
        icp->setTransformationEpsilon(1e-4);
        icp->setEuclideanFitnessEpsilon(1);
        icp->setRANSACOutlierRejectionThreshold(2 * resolution);

        globalized_alignment(rough_clouds, icp, 1);

//        std::vector<Eigen::Matrix4f> propagate(n);
//        std::fill(propagate.begin(), propagate.end(), Eigen::Matrix4f::Identity());

//        for (int i = 1; i < n; ++i)
//        {
//            auto source = sorted_nodes[i];
//            auto target = path[source];

//            pcl::transformPointCloud(*rough_clouds[source], *rough_clouds[source], propagate[target]);
//            propagate[source] = propagate[target] * propagate[source];

//            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//            icp.setMaxCorrespondenceDistance (100 * resolution);
//            icp.setMaximumIterations(100);
//            icp.setTransformationEpsilon(1e-4);
//            icp.setEuclideanFitnessEpsilon(1);
//            icp.setRANSACOutlierRejectionThreshold(resolution);
//            icp.setInputSource(rough_clouds[source]);
//            icp.setInputTarget(rough_clouds[target]);
//            pcl::PointCloud<pcl::PointXYZ> cloud;
//            icp.align(cloud);

//            pcl::copyPointCloud(cloud, *rough_clouds[source]);
//            propagate[source] = icp.getFinalTransformation() * propagate[source];
//        }
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << "took " << std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count() << " secs" << std::endl;
    }

    // FINE-ALIGNMENT
    {
        std::cout << "aligning (fine) point rough_clouds... ";
        auto start = std::chrono::high_resolution_clock::now();

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr icp(new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>);
        icp->setMaxCorrespondenceDistance (75 * resolution);
        icp->setMaximumIterations(100);
        icp->setTransformationEpsilon(1e-8);
        icp->setEuclideanFitnessEpsilon(1e-8);
        icp->setRANSACOutlierRejectionThreshold(2 * resolution);
        icp->setUseReciprocalCorrespondences(true);

        globalized_alignment(rough_clouds, icp, 1);

//        std::vector<Eigen::Matrix4f> propagate(n);
//        std::fill(propagate.begin(), propagate.end(), Eigen::Matrix4f::Identity());

//        for (int i = 1; i < n; ++i)
//        {
//            auto source = sorted_nodes[i];
//            auto target = path[source];

//            pcl::transformPointCloud(*rough_clouds[source], *rough_clouds[source], propagate[target]);
//            propagate[source] = propagate[target] * propagate[source];

//            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//            icp.setMaxCorrespondenceDistance (75 * resolution);
//            icp.setUseReciprocalCorrespondences(true);
//            icp.setTransformationEpsilon (1e-8);
//            icp.setEuclideanFitnessEpsilon (1e-8);
//            icp.setRANSACOutlierRejectionThreshold(resolution);
//            icp.setInputSource(rough_clouds[source]);
//            icp.setInputTarget(rough_clouds[target]);

//            pcl::PointCloud<pcl::PointXYZ> cloud;
//            icp.align(cloud);

//            pcl::copyPointCloud(cloud, *rough_clouds[source]);
//            propagate[source] = icp.getFinalTransformation() * propagate[source];
//        }
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << "took " << std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count() << " secs" << std::endl;
    }

    // CALCULATE FINAL ERROR
    {
        float error = 0.0f;

        std::cout << "calculating final error... ";
        auto start = std::chrono::high_resolution_clock::now();

        for (int source = 0; source < n; ++source)
        {
            int target = path[source];
            error += calculate_error<pcl::PointXYZ>(rough_clouds[source], rough_clouds[target], correspondences[source]);

        }

        auto end = std::chrono::high_resolution_clock::now();
        std::cout << "took " << std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count() << " secs" << std::endl;
        std::cout << "error: " << error << std::endl;
    }

    // VISUALIZE FINAL ALIGNMENT
    {
        pcl::visualization::PCLVisualizer viewer;
        for (int i = 0; i < n; ++i)
        {
            pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> handler(rough_clouds[i]);
            viewer.addPointCloud(rough_clouds[i], handler, std::to_string(i));
        }
        viewer.spin();
    }

    return 0;
}
