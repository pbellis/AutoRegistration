#ifndef OPENNI2HANDLER_H
#define OPENNI2HANDLER_H

#include <QObject>
#include <QTime>

#include <OpenNI.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class OpenNI2Handler : public QObject, openni::VideoStream::NewFrameListener
{
    Q_OBJECT
public:
    explicit OpenNI2Handler(QObject *parent = 0);
    virtual ~OpenNI2Handler();

    bool isRunning() const;
signals:
    void messagePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                           const pcl::PointCloud<pcl::Normal>::Ptr &normals,
                           const std::string &name);
public slots:
    void onInitialize() { initialize(); }
    void onCreate() { create(); }
    void onToggle() { if (running = !running) start(); else stop(); }
private /*methods*/ :
    void initialize();
    void create();
    void start();
    void stop();
    void extract();
private /*members*/ :
    bool running;
    QTime time;
    openni::Status status;
    openni::Device device;

    openni::VideoStream depthStream;
    openni::VideoFrameRef depthFrame;

    // NewFrameListener interface
public:
    virtual void onNewFrame(openni::VideoStream &);
};

#endif // OPENNI2HANDLER_H
