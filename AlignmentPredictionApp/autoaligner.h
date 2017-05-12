#ifndef AUTOALIGNER_H
#define AUTOALIGNER_H

#include <QObject>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

class AutoAligner : public QObject
{
    Q_OBJECT
public:
    explicit AutoAligner(QObject *parent = 0);
signals:
    void messageAlignmentStrategy(const Eigen::MatrixXd &distance_matrix, const Eigen::VectorXi &path);
    void messagePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::string &name);
    void messageFinalized();
public slots:
    void onAddPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::string &name);
    void onFinalizeAlignment();
private /*members*/ :
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> lowresClouds;
private /*methods*/:
    void incrementalAlign();
    void finalAlign();
};

#endif // AUTOALIGNER_H
