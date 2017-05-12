#include "autoaligner.h"
#include <pcl/registration/icp.h>
#include <QDebug>

#include <pcl/filters/filter.h>
#include <pcl/keypoints/uniform_sampling.h>

#include "graph_theory.h"
#include "statistical_shape.h"
#include "covariance.h"
#include "mean.h"

#include "statistical_distance.h"


AutoAligner::AutoAligner(QObject *parent) : QObject(parent)
{
}

void AutoAligner::onAddPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::string &)
{
    qDebug() << Q_FUNC_INFO;
    pcl::PointCloud<pcl::PointXYZ>::Ptr nonans(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*cloud, *nonans, index);

    if (nonans->size() < 300)
    {
        qDebug() << "ignoring cloud, too small!";
        return;
    }

    clouds.push_back(nonans);

    pcl::PointCloud<int> indices;

    pcl::UniformSampling<pcl::PointXYZ> us;
    us.setInputCloud(nonans);
    us.setKSearch(5);
    us.compute(indices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr lowresCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*nonans, indices.points, *lowresCloud);
    lowresClouds.push_back(lowresCloud);

    if (clouds.size() > 1) incrementalAlign();
    emit messagePointCloud(clouds.back(), std::to_string(clouds.size() - 1));
}

void AutoAligner::onFinalizeAlignment()
{
    emit messageFinalized();
    finalAlign();
}

void AutoAligner::incrementalAlign()
{
    qDebug() << Q_FUNC_INFO;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//    icp.setMaximumIterations(10);
    icp.setInputSource(*(lowresClouds.end() - 1));
    icp.setInputTarget(*(lowresClouds.end() - 2));
    pcl::PointCloud<pcl::PointXYZ> cloud;
    icp.align(cloud);
    pcl::transformPointCloud(**(clouds.end() - 1), **(clouds.end() - 1), icp.getFinalTransformation());
}

#include <pcl/common/centroid.h>

void AutoAligner::finalAlign()
{
    qDebug() << Q_FUNC_INFO;
    const size_t N = clouds.size();

    // CALCULATE MEAN (OR CENTROID) FOR EACH SCAN
    std::vector<Eigen::Vector3d> means(N);
    std::transform(clouds.cbegin(), clouds.cend(), means.begin(), [](const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        Eigen::VectorXd meanX;
        calculate_mean(meanX, cloud->getMatrixXfMap().cast<double>());
        Eigen::Vector3d mean(meanX[0], meanX[1], meanX[2]);
        return mean;

    });

    // CALCULATE COVARIANCE MATRIX FOR EACH SCAN
    std::vector<Eigen::Matrix3d> covariances(N);
    std::transform(clouds.cbegin(), clouds.cend(), covariances.begin(), [](const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        Eigen::VectorXd m; Eigen::Matrix3d covariance;
        //calculate_mean_and_covariance(m, covariance, cloud->getMatrixXfMap().cast<double>());

        return covariance;
    });
    qDebug() << "calculated mean value and covariance matrix of each scan...";

    // CALCULATE STATISTICAL DISTANCE BETWEEN EACH SCAN PAIR
    Eigen::MatrixXd distance_matrix(N, N);
    //calculate_multivariate_hellinger_distance(distance_matrix, means, covariances);
    qDebug() << "calculated pair wise distances for all scans";

    // CALCULATE THE MINIMUM SPANNING TREE OF THAT DISTANCE MATRIX
    double total_distance; Eigen::VectorXi minimum_spanning_tree(N);
    calculate_minimum_spanning_tree(total_distance, minimum_spanning_tree, distance_matrix);
    qDebug() << "calculated minimum spanning tree...";

    // CALCULATE THE DEGREE OF EACH SCAN
    Eigen::VectorXi node_degree(N);
    calculate_degree(node_degree, minimum_spanning_tree);
    qDebug() << "calculated node degrees...";

    // CALCULATE THE BALANCE OF THE GRAPH
    Eigen::MatrixXd balance_matrix(N, N);
    calculate_graph_balance(balance_matrix, node_degree, total_distance, minimum_spanning_tree, distance_matrix);
    qDebug() << "calculated graph balance point...";

    // CALCULATE THE CENTER SCAN OF THE GRAPH (THIS IS WHAT WE WILL TRY TO ALIGN ALL SCANS TO)
    int center;
    calculate_graph_center(center, balance_matrix, minimum_spanning_tree);
    qDebug() << "calculated center of graph...";

    // CALCULATE WHICH SCANS SHOULD ALIGN TO WHICH SCANS
    Eigen::VectorXi path(N);
    calculate_optimal_path(path, center, minimum_spanning_tree, node_degree);
    qDebug() << "calculated optimal path for alignment...";

    emit messageAlignmentStrategy(distance_matrix, path);

    Eigen::VectorXi sorted_nodes(N);
    std::iota(sorted_nodes.data(), sorted_nodes.data() + N, 0);
    std::sort(sorted_nodes.data(), sorted_nodes.data() + N, [&](int i, int j){
        return node_degree[i] < node_degree[j];
    });

    std::vector<Eigen::Matrix4f> propagate(N);
    std::fill(propagate.begin(), propagate.end(), Eigen::Matrix4f::Identity());

    for (int i = N-1; i >= 0; --i)
    {
        auto source = sorted_nodes[i];
        auto target = path[source];

        qDebug() << "aligning scan" << source << "to" << target;
        qDebug() << (N-i) << "/" << N;

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setMaxCorrespondenceDistance (0.05);
        icp.setMaximumIterations (50);
        icp.setTransformationEpsilon (1e-8);
        icp.setEuclideanFitnessEpsilon (1);
        icp.setInputSource(clouds[source]);
        icp.setInputTarget(clouds[target]);
        pcl::PointCloud<pcl::PointXYZ> cloud;
        icp.align(cloud, propagate[target]);

        propagate[source] *= icp.getFinalTransformation();
        pcl::copyPointCloud(cloud, *clouds[source]);

        emit messagePointCloud(clouds[source], std::to_string(sorted_nodes[i]));
    }
}
