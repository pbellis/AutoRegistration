#include <pcl/features/integral_image_normal.h>

#include "openni2handler.h"
#include <QDebug>
#include <QApplication>


OpenNI2Handler::OpenNI2Handler(QObject *parent) : QObject(parent)
{
    running = false;
    time = QTime::currentTime();
}

OpenNI2Handler::~OpenNI2Handler()
{
    stop();
    openni::OpenNI::shutdown();
}

bool OpenNI2Handler::isRunning() const
{
    return running;
}

void OpenNI2Handler::initialize()
{
    qDebug() << Q_FUNC_INFO;
    status = openni::STATUS_OK;

    const char* deviceURI = openni::ANY_DEVICE;

    status = openni::OpenNI::initialize();
    status = device.open(deviceURI);
    if (status != openni::STATUS_OK)
    {
        qDebug() << QString(openni::OpenNI::getExtendedError());
        openni::OpenNI::shutdown();
        QApplication::quit();
    }
}

void OpenNI2Handler::create()
{
    qDebug() << Q_FUNC_INFO;
    status = depthStream.create(device, openni::SENSOR_DEPTH);
    if (status != openni::STATUS_OK)
    {
        qDebug() << "error creating depth stream...";
        qDebug() << QString(openni::OpenNI::getExtendedError());
        openni::OpenNI::shutdown();
        QApplication::quit();
    }
    else
    {
        status = depthStream.addNewFrameListener(this);
    }
}

void OpenNI2Handler::start()
{
    qDebug() << Q_FUNC_INFO;
    status = depthStream.start();
}

void OpenNI2Handler::stop()
{
    qDebug() << Q_FUNC_INFO;
    depthStream.stop();
}

void OpenNI2Handler::extract()
{
    qDebug() << Q_FUNC_INFO;
    // ENSURE WE DO NOT STARVE SIGNALS
    QCoreApplication::processEvents();

    depthStream.readFrame(&depthFrame);

    auto pDepthRow = (const openni::DepthPixel*) depthFrame.getData();
    const auto height = depthFrame.getHeight();
    const auto width = depthFrame.getWidth();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->height = height;
    cloud->width = width;
    cloud->is_dense = false;

    cloud->points.resize(height * width);

    int index = 0;
    for (auto j = 0; j < height; ++j)
    {
        for (auto i = 0; i < width; ++i, ++index)
        {
            const auto &depth = pDepthRow[index];
            if (depth == 0)
            {
                cloud->points[index].x = cloud->points[index].y = cloud->points[index].z = std::numeric_limits<float>::quiet_NaN ();
            }
            else
            {
                openni::CoordinateConverter::convertDepthToWorld(depthStream, i, j, depth, &cloud->points[index].x, &cloud->points[index].y, &cloud->points[index].z);
            }
        }
    }

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
    ne.setBorderPolicy(ne.BORDER_POLICY_MIRROR);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);

    emit messagePointCloud(cloud, normals, std::to_string(depthFrame.getFrameIndex()));
}

void OpenNI2Handler::onNewFrame(openni::VideoStream &)
{
    qDebug() << Q_FUNC_INFO;
    extract();
}
