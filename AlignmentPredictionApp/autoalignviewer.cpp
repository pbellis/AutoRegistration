#include "autoalignviewer.h"
#include "../build/AlignmentPredictionApp/ui_autoalignviewer.h"
#include "openni2handler.h"
#include "autoaligner.h"
#include <QThread>
#include <QDebug>
#include "distancematrixvisualizer.h"

AutoAlignViewer::AutoAlignViewer(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::AutoAlignViewer),
    handler_thread(new QThread),
    handler(new OpenNI2Handler),
    aligner_thread(new QThread),
    aligner(new AutoAligner)
{
    ui->setupUi(this);
    this->setWindowTitle ("Auto Align Viewer");

    // SET UP OPENNI2 HANDLER
    handler->onInitialize();
    handler->onCreate();
    handler->onToggle();

    // SET OPENNI2 HANDLER THREAD
    handler->moveToThread(handler_thread);
    handler_thread->start();

    // SET AUTO ALIGNER THREAD
    aligner->moveToThread(aligner_thread);
    aligner_thread->start();

    // SET UP ON/OFF FOR RECORD MODE
    connect(ui->toggleCaputureButton, &QPushButton::toggled,
            this, &AutoAlignViewer::onToggle);

    // SET UP CURRENT FRAME DISPLAY
    ui->currentFrameViewer->onSetSingleFrameViewer(true);
    connect(handler, &OpenNI2Handler::messagePointCloud,
            ui->currentFrameViewer, &PCLViewer::onAddPointCloudWithNormals);

    // SET UP GLOBAL ICP ALGORITHM
    connect(ui->finalizeButton, &QPushButton::clicked,
            aligner, &AutoAligner::onFinalizeAlignment);
    connect(aligner, &AutoAligner::messageFinalized,
            ui->processedViewer, &PCLViewer::onClearAll);
    connect(aligner, &AutoAligner::messagePointCloud,
            ui->processedViewer, &PCLViewer::onAddPointCloud);
    connect(aligner, &AutoAligner::messageAlignmentStrategy,
            this, &AutoAlignViewer::onRecieveAlignmentStrategy);
}

AutoAlignViewer::~AutoAlignViewer()
{
    delete handler_thread;
    delete handler;
    delete aligner;
    delete ui;
}


void AutoAlignViewer::onToggle(bool toggle)
{
    qDebug() << Q_FUNC_INFO;
//    if (toggle)
//    {
//        connect(handler, &OpenNI2Handler::messagePointCloud,
//                aligner, &AutoAligner::onAddPointCloud);
//    }
//    else
//    {
//        disconnect(handler, &OpenNI2Handler::messagePointCloud,
//                aligner, &AutoAligner::onAddPointCloud);
//    }
}

void AutoAlignViewer::onRecieveAlignmentStrategy(const Eigen::MatrixXd &distance_matrix, const Eigen::VectorXi &path)
{
    DistanceMatrixVisualizer* dmv = new DistanceMatrixVisualizer;
    dmv->onRecieveAlignmentStrategy(distance_matrix, path);
    dmv->show();
}
