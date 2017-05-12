#include "input_handler.h"

#include <QFile>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <iostream>

std::vector<PoseMetaInfo> read_conf_stream(QTextStream &in)
{
    std::vector<PoseMetaInfo> pose_meta;

    while (!in.atEnd())
    {
        QString line = in.readLine();
        QStringList split = line.split(' ');

        if (QString::compare(split[0], "bmesh", Qt::CaseInsensitive) == 0)
        {
            float Tx = split.at(2).toFloat();
            float Ty = split.at(3).toFloat();
            float Tz = split.at(4).toFloat();

            float Qx = split.at(5).toFloat();
            float Qy = split.at(6).toFloat();
            float Qz = split.at(7).toFloat();
            float Qw = split.at(8).toFloat();

            float s = 2.0 / (Qx*Qx + Qy*Qy + Qz*Qz + Qw*Qw);

            float xs = Qx * s;
            float ys = Qy * s;
            float zs = Qz * s;

            float wx = Qw * xs;
            float wy = Qw * ys;
            float wz = Qw * zs;

            float xx = Qx * xs;
            float xy = Qx * ys;
            float xz = Qx * zs;

            float yy = Qy * ys;
            float yz = Qy * zs;
            float zz = Qz * zs;

            Eigen::Matrix4f transform;
            transform <<
                1.0 - (yy + zz), xy + wz, xz - wy, Tx,
                xy - wz, 1 - (xx + zz), yz + wx, Ty,
                xz + wy, yz - wx, 1 - (xx + yy), Tz,
                0.0f, 0.0f, 0.f, 1.0f;

            pose_meta.push_back(PoseMetaInfo{split.at(1), transform});
        }
    }
    return pose_meta;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> load_ply_files(const QString &directory, const QString &conf_filepath)
{
    QFile conf_file(conf_filepath);
    conf_file.open(QIODevice::ReadOnly | QIODevice::Text);
    QTextStream in(&conf_file);
    std::vector<PoseMetaInfo> pose_meta = read_conf_stream(in);
    conf_file.close();

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
    clouds.resize(pose_meta.size());

    std::transform(pose_meta.cbegin(), pose_meta.cend(), clouds.begin(), [&](const PoseMetaInfo& meta)
    {
        QString filepath = directory + QString("\\") + meta.name;

        std::cout << "loading " << filepath.toStdString() << " ... ";
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile(filepath.toStdString(), *cloud);
        pcl::transformPointCloud<pcl::PointXYZ>(*cloud, *cloud, meta.transform);
        std::cout << "has " << cloud->size() << " points" << std::endl;
        return cloud;
    });

    return clouds;
}
