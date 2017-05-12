#include "pclviewer.h"
#include "../build/AlignmentPredictionApp/ui_pclviewer.h"
#include <QDebug>
#include "pointcloudnormalhandler.h"

PCLViewer::PCLViewer (QWidget *parent) :
  QWidget (parent),
  ui (new Ui::PCLViewer)
{
  ui->setupUi (this);
  this->setWindowTitle ("PCL viewer");
  singleFrameViewer = false;

  // Set up the QVTK window
  viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  ui->centralwidget->SetRenderWindow (viewer->getRenderWindow ());
  viewer->setupInteractor (ui->centralwidget->GetInteractor (), ui->centralwidget->GetRenderWindow ());
  ui->centralwidget->update();
}

PCLViewer::~PCLViewer ()
{
    delete ui;
}

void PCLViewer::onAddPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::string &name)
{
    qDebug() << Q_FUNC_INFO;
    if (singleFrameViewer) viewer->removeAllPointClouds();
    viewer->addPointCloud<pcl::PointXYZ>(cloud, name);
    onUpdate();
}

void PCLViewer::onAddPointCloudWithNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const pcl::PointCloud<pcl::Normal>::Ptr &normals, const std::string &name)
{
    qDebug() << Q_FUNC_INFO;
    if (singleFrameViewer) viewer->removeAllPointClouds();
    pcl::visualization::PointCloudNormalHandler<pcl::PointXYZ> handler(cloud, normals);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, handler, name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
    onUpdate();
}

void PCLViewer::onUpdate()
{
    qDebug() << Q_FUNC_INFO;
    viewer->resetCamera ();
    ui->centralwidget->update ();
}

void PCLViewer::onClearAll()
{
    viewer->removeAllPointClouds();
}

void PCLViewer::onSetSingleFrameViewer(bool single)
{
    qDebug() << Q_FUNC_INFO;
    singleFrameViewer = single;
}
