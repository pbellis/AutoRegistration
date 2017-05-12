#ifndef PCLVIEWER_H
#define PCLVIEWER_H

#include <iostream>

// Qt
#include <QMainWindow>
#include <QPushButton>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

namespace Ui
{
  class PCLViewer;
}

class PCLViewer : public QWidget
{
  Q_OBJECT

public:
  explicit PCLViewer (QWidget *parent = 0);
  ~PCLViewer ();
public slots:
    void onAddPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::string &name);
    void onAddPointCloudWithNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                    const pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                    const std::string &name);
    void onUpdate();
    void onClearAll();
    void onSetSingleFrameViewer(bool single);

protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

private:
  Ui::PCLViewer *ui;
  bool singleFrameViewer;

};

#endif // PCLVIEWER_H
