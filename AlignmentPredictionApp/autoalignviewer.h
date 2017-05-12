#ifndef AUTOALIGNVIEWER_H
#define AUTOALIGNVIEWER_H

#include <QWidget>
#include <QThread>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace Ui {
class AutoAlignViewer;
}

class OpenNI2Handler;
class AutoAligner;

class AutoAlignViewer : public QWidget
{
    Q_OBJECT

public:
    explicit AutoAlignViewer(QWidget *parent = 0);
    ~AutoAlignViewer();

private /*members*/ :
    Ui::AutoAlignViewer *ui;
    QThread* handler_thread;
    OpenNI2Handler *handler;
    QThread* aligner_thread;
    AutoAligner* aligner;
private slots:
    void onToggle(bool toggle);
    void onRecieveAlignmentStrategy(const Eigen::MatrixXd &distance_matrix, const Eigen::VectorXi &path);
};

#endif // AUTOALIGNVIEWER_H
