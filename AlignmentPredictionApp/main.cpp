#include <iostream>
#include <string>
#include <random>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/keypoints/uniform_sampling.h> // possible use in getting rid of extra data
#include <pcl/common/pca.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/filter.h>

#include "graph_theory.h"
#include "statistical_shape.h"
#include "statistical_distance.h"

#include <typeinfo>

#include "distancematrixvisualizer.h"
#include "autoalignviewer.h"
#include "pclviewer.h"
#include <pcl/io/openni2_grabber.h>
#include "openni2handler.h"
#include <QApplication>
#include <QMainWindow>

#include <QDebug>
#include <QString>
#include <QThread>

Q_DECLARE_METATYPE(pcl::PointCloud<pcl::PointXYZ>::Ptr)
Q_DECLARE_METATYPE(pcl::PointCloud<pcl::Normal>::Ptr)
Q_DECLARE_METATYPE(std::string)
Q_DECLARE_METATYPE(Eigen::MatrixXd)
Q_DECLARE_METATYPE(Eigen::VectorXi)

int main(int argc, char *argv[])
{
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    QApplication a(argc, argv);

    qRegisterMetaType<pcl::PointCloud<pcl::PointXYZ>::Ptr>("pcl::PointCloud<pcl::PointXYZ>::Ptr");
    qRegisterMetaType<pcl::PointCloud<pcl::PointXYZ>::Ptr>("pcl::PointCloud<pcl::Normal>::Ptr");
    qRegisterMetaType<std::string>("std::string");
    qRegisterMetaType<Eigen::MatrixXd>("Eigen::MatrixXd");
    qRegisterMetaType<Eigen::VectorXi>("Eigen::VectorXi");

    AutoAlignViewer* autoAlignViewer = new AutoAlignViewer;

    autoAlignViewer->show();
    auto r = a.exec();
    openni::OpenNI::shutdown();
    return r;
}


//int nothing(int argc, char *argv[])
//{
//    // DISABLE ALL OUTPUT FROM PCL
//    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

//   // SHUFFLE SCANS TO RANDOMIZE DATA
//   std::vector<std::string> files(argv + 1, argv + argc);
//   auto engine = std::default_random_engine{};
//   std::shuffle(files.begin(), files.end(), engine);
//   std::shuffle(files.begin(), files.end(), engine);
//   std::shuffle(files.begin(), files.end(), engine);

//   const size_t N = files.size();

//   // LOAD ALL SCANS IN (MUST BE PCD FORMAT)
//   std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds(N);
//   std::transform(files.cbegin(), files.cend(), clouds.begin(), [](const std::string &file)
//   {
//       std::cout << "loading " << file << std::endl;
//       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//       //pcl::io::loadPCDFile(file, *cloud);
//       pcl::io::loadPLYFile(file, *cloud);

//       pcl::PointCloud<pcl::PointXYZ>::Ptr nonans(new pcl::PointCloud<pcl::PointXYZ>);
//       std::vector<int> index;
//       pcl::removeNaNFromPointCloud(*cloud, *nonans, index);

//       std::cout << "file has " << nonans->size() << " points" << std::endl;

//       return nonans;
//   });

//   /*
//   // VIEW STATISTICAL SHAPE OF CLOUD
//   std::for_each(clouds.cbegin(), clouds.cend(), [](const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
//   {
//       pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//       kdtree.setInputCloud(cloud);
//       std::vector<StatisticalShape> statistical_shape;
//       calcuate_statistical_shape(statistical_shape, *cloud, kdtree, 0.01);

//       pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>);
//       colored->resize(cloud->size());

//       auto minmaxOmnivariance = std::minmax_element(statistical_shape.cbegin(), statistical_shape.cend(),
//        [] (const StatisticalShape &a, const StatisticalShape &b)
//        {
//            return a.cubed_omnivariance < b.cubed_omnivariance;
//        });

//       float rangeOmnivariance = minmaxOmnivariance.second->cubed_omnivariance - minmaxOmnivariance.first->cubed_omnivariance;

//       auto minmaxAnisotrophy = std::minmax_element(statistical_shape.cbegin(), statistical_shape.cend(),
//        [] (const StatisticalShape &a, const StatisticalShape &b)
//        {
//            return a.anisotropy < b.anisotropy;
//        });

//       float rangeAnisotrophy = minmaxAnisotrophy.second->anisotropy - minmaxAnisotrophy.first->anisotropy;

//       auto minmaxCurvatureChange = std::minmax_element(statistical_shape.cbegin(), statistical_shape.cend(),
//       [] (const StatisticalShape &a, const StatisticalShape &b)
//       {
//           return a.curvature_change < b.curvature_change;
//       });

//       float rangeCurvatureChange = minmaxCurvatureChange.second->curvature_change - minmaxCurvatureChange.first->curvature_change;

//       for (auto i = 0; i < statistical_shape.size(); ++i)
//       {
//           colored->points[i].x = cloud->points[i].x;
//           colored->points[i].y = cloud->points[i].y;
//           colored->points[i].z = cloud->points[i].z;
//           colored->points[i].r = 255 * ((statistical_shape[i].curvature_change - minmaxCurvatureChange.first->curvature_change) / rangeCurvatureChange);
//           colored->points[i].g = 255 * ((statistical_shape[i].anisotropy - minmaxAnisotrophy.first->anisotropy) / rangeAnisotrophy);
//           colored->points[i].b = 255 * ((statistical_shape[i].cubed_omnivariance - minmaxAnisotrophy.first->cubed_omnivariance) / rangeOmnivariance);
//       }

//       pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(colored);

//       pcl::visualization::PCLVisualizer pcl_visualizer;
//       pcl_visualizer.addPointCloud<pcl::PointXYZRGB> (colored, rgb, "cloud");
//       pcl_visualizer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

//       while (!pcl_visualizer.wasStopped())
//       {
//           pcl_visualizer.spinOnce();
//       }
//   });
//*/

//   // CALCULATE MEAN (OR CENTROID) FOR EACH SCAN
//   std::vector<Eigen::Vector3d> means(N);
//   std::transform(clouds.cbegin(), clouds.cend(), means.begin(), [](const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
//   {
//       Eigen::Vector3d mean;
//       calculate_mean(mean, cloud->getMatrixXfMap().cast<double>());
//       return mean;
//   });

//   // CALCULATE COVARIANCE MATRIX FOR EACH SCAN
//   std::vector<Eigen::Matrix3d> covariances(N);
//   std::transform(clouds.cbegin(), clouds.cend(), covariances.begin(), [](const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
//   {
//       Eigen::Vector3d mean; Eigen::Matrix3d covariance;
//       calculate_mean_and_covariance(mean, covariance, cloud->getMatrixXfMap().cast<double>());
//       return covariance;
//   });

//   // CALCULATE STATISTICAL DISTANCE BETWEEN EACH SCAN PAIR
//   Eigen::MatrixXd distance_matrix(N, N);
//   calculate_multivariate_hellinger_distance(distance_matrix, means, covariances);
//   std::cout << "distance_matrix" << std::endl << distance_matrix << std::endl;

//   // CALCULATE THE MINIMUM SPANNING TREE OF THAT DISTANCE MATRIX
//   double total_distance; Eigen::VectorXi minimum_spanning_tree(N);
//   calculate_minimum_spanning_tree(total_distance, minimum_spanning_tree, distance_matrix);

//   std::cout << "total_distance " << total_distance << std::endl;
//   std::cout << "minimum_spanning_tree " << minimum_spanning_tree.transpose() << std::endl;

//   // CALCULATE THE DEGREE OF EACH SCAN
//   Eigen::VectorXi node_degree(N);
//   calculate_degree(node_degree, minimum_spanning_tree);

//   std::cout << "node_degree " << node_degree.transpose() << std::endl;

//   // CALCULATE THE BALANCE OF THE GRAPH
//   Eigen::MatrixXd balance_matrix(N, N);
//   calculate_graph_balance(balance_matrix, node_degree, total_distance, minimum_spanning_tree, distance_matrix);

//   std::cout << "balance_matrix " << std::endl << balance_matrix.transpose() << std::endl;

//   // CALCULATE THE CENTER SCAN OF THE GRAPH (THIS IS WHAT WE WILL TRY TO ALIGN ALL SCANS TO)
//   int center;
//   calculate_graph_center(center, balance_matrix, minimum_spanning_tree);

//   std::cout << "center " << center << std::endl;

//   // CALCULATE WHICH SCANS SHOULD ALIGN TO WHICH SCANS
//   Eigen::VectorXi path(N);
//   calculate_optimal_path(path, center, minimum_spanning_tree, node_degree);

//   std::cout << "path " << path.transpose() << std::endl;

//   for (size_t i = 0; i < N; ++i)
//   {
//       const size_t j = path[i];

//       // THE BELOW FUNCTION FINDS THE "PROBABILITY" THAT A POINT IS IN AN OVERLAPPING REGION
//       // TO FIND AN APPROXIMATE BOUNDARY JUST TAKE POINTS GREATER THAN SOME THRESHOLD PROBABILITY
//       std::vector<double> source_ownership_uncertainty;
//       calculate_ownership_uncertainty(source_ownership_uncertainty, means[i], covariances[i], *clouds[i], means[j], covariances[j], clouds[j]->size());

//       // THE BELOW FUNCTION FINDS POINTS THAT FOLLOW A MORE EXACT BOUNDARY OF THE OVERLAPPING REGION
//       std::vector<int> source_uncertain_indices;
//       find_uncertain_region(source_uncertain_indices, source_ownership_uncertainty, *clouds[i], 1 - distance_matrix(i, j));

//       pcl::PointCloud<pcl::PointXYZ>::Ptr source_region(new pcl::PointCloud<pcl::PointXYZ>);
//       pcl::copyPointCloud(*clouds[i], source_uncertain_indices, *source_region);

//       pcl::visualization::PCLVisualizer pcl_visualizer;

//       pcl_visualizer.addPointCloud(clouds[i], "source_cloud");
//       pcl_visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "source_cloud");

//       pcl_visualizer.addPointCloud(clouds[j], "target_cloud");
//       pcl_visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "target_cloud");

//       pcl_visualizer.addPointCloud(source_region, "source_region");
//       pcl_visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 1, "source_region");
//       pcl_visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source_region");

//       while (!pcl_visualizer.wasStopped())
//       {
//           pcl_visualizer.spinOnce();
//       }
//   }
//   return 0;
//}

