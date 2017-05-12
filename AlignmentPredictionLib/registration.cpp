#include "registration.h"
#include <iostream>
bool globalized_alignment(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_vec, pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr &icp, int iters)
{
    const auto n = cloud_vec.size();

    // CALCULATE MEAN VECTOR OF EACH CLOUD
    std::vector<Eigen::VectorXd> mean_vec(n);
    std::transform(cloud_vec.cbegin(), cloud_vec.cend(), mean_vec.begin(), [](const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        Eigen::VectorXd mean;
        calculate_mean(mean, cloud->getMatrixXfMap().topRows(3).cast<double>());
        return mean;
    });

    // CALCULATE COVARIANCE MATRIX OF EACH CLOUD
    std::vector<Eigen::MatrixXd> covariance_vec(n);
    std::transform(cloud_vec.cbegin(), cloud_vec.cend(), mean_vec.cbegin(), covariance_vec.begin(), [](const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const Eigen::VectorXd &mean)
    {
        Eigen::MatrixXd covariance;
        calculate_covariance(covariance, cloud->getMatrixXfMap().topRows(3).cast<double>(), mean);

        return covariance;
    });

    // INITIALIZE PROPAGATION TRANSFORMATION MATRICES
    std::vector<Eigen::Matrix4f> propagate(n);
    pcl::PointCloud<pcl::PointXYZ>::Ptr icp_dump(new pcl::PointCloud<pcl::PointXYZ>);

    bool converged;
    int iter = 0;
    do
    {
        converged = true;
        iter++;

        std::cout << "iteration " << iter << std::endl;

        // CALCULATE MULTIVARIATE HELLINGER DISTANCE BETWEEN EACH CLOUD
        Eigen::MatrixXd distance_matrix(n, n);
        calculate_multivariate_hellinger_distance(distance_matrix, mean_vec, covariance_vec);

        std::cout << distance_matrix << std::endl;

        // CALCULATE MINIMUM SPANNING TREE THEN TOTAL DISTANCE ACROSS THE MINIMUM SPANNING TREE
        double total_distance; Eigen::VectorXi minimum_spanning_tree(n);
        calculate_minimum_spanning_tree(total_distance, minimum_spanning_tree, distance_matrix);

        std::cout << "mst: " << minimum_spanning_tree.transpose() << std::endl;

        // CALCULATE THE DEGREE OF EACH NODE
        Eigen::VectorXi node_degree(n);
        calculate_degree(node_degree, minimum_spanning_tree);

        std::cout << "degree: " << node_degree.transpose() << std::endl;

        // CALCULATE THE BALANCE OF THE GRAPH
        Eigen::MatrixXd balance_matrix(n, n);
        calculate_graph_balance(balance_matrix, node_degree, total_distance, minimum_spanning_tree, distance_matrix);

        // CALCULATE THE CENTER SCAN OF THE GRAPH (THIS IS WHAT WE WILL TRY TO ALIGN ALL SCANS TO)
        int center;
        calculate_graph_center(center, balance_matrix, minimum_spanning_tree);

        // CALCULATE WHICH SCANS SHOULD ALIGN TO WHICH SCANS
        Eigen::VectorXi path(n);
        calculate_optimal_path(path, center, minimum_spanning_tree, node_degree);

        std::cout << "path: " << path.transpose() << std::endl;

        Eigen::VectorXi node_distances(n);
        calculate_node_distances(node_distances, path, node_degree, center);

        // SORT NODES BY DISTANCE TO CENTER
        Eigen::VectorXi sorted_nodes(n);
        sort_nodes_by_distance(sorted_nodes, node_distances);

        std::cout << "nodes: " << sorted_nodes.transpose() << std::endl;

        std::fill(propagate.begin(), propagate.end(), Eigen::Matrix4f::Identity());
        for (int i = 1; i < n; ++i)
        {
            auto source = sorted_nodes[i];
            auto target = path[source];

            pcl::transformPointCloud(*cloud_vec[source], *cloud_vec[source], propagate[target]);
            propagate[source] = propagate[target] * propagate[source];

            icp->setInputSource(cloud_vec[source]);
            icp->setInputTarget(cloud_vec[target]);
            icp->align(*icp_dump);

            pcl::copyPointCloud(*icp_dump, *cloud_vec[source]);
            propagate[source] = icp->getFinalTransformation() * propagate[source];

            calculate_mean(mean_vec[source], cloud_vec[source]->getMatrixXfMap().topRows(3).cast<double>());
            calculate_covariance(covariance_vec[source], cloud_vec[source]->getMatrixXfMap().topRows(3).cast<double>(), mean_vec[source]);

            converged &= icp->hasConverged();
        }
    } while (iter < iters);
//!converged &&
    return converged;
}
