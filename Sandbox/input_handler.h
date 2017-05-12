#ifndef INPUT_HANDLER_H
#define INPUT_HANDLER_H

#include <QString>
#include <QDirIterator>
#include <QTextStream>

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

struct PoseMetaInfo
{
    QString name;
    Eigen::Matrix4f transform;
};

std::vector<PoseMetaInfo> read_conf_stream(QTextStream &in);

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> load_ply_files(const QString &directory, const QString &conf_filepath);

#endif // INPUT_HANDLER_H
