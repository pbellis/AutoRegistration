#include "distancematrixvisualizer.h"
#include "../build/AlignmentPredictionApp/ui_distancematrixvisualizer.h"
#include <QImage>
#include <QDebug>

DistanceMatrixVisualizer::DistanceMatrixVisualizer(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DistanceMatrixVisualizer)
{
    ui->setupUi(this);
}

DistanceMatrixVisualizer::~DistanceMatrixVisualizer()
{
    delete ui;
}

void DistanceMatrixVisualizer::onRecieveAlignmentStrategy(const Eigen::MatrixXd &distance_matrix, const Eigen::VectorXi &path)
{
    qDebug() << Q_FUNC_INFO;
    auto cols = distance_matrix.cols();
    auto rows = distance_matrix.rows();

    auto max_coef = distance_matrix.maxCoeff();

    QImage image(cols, rows, QImage::Format_RGB32);
    for (auto row = 0; row < rows; ++row)
    {
        auto line = (QRgb*) image.scanLine(row);
        for (auto col = 0; col < cols; ++col)
        {
            line[col] = qRgb((distance_matrix(col, row) / max_coef) * 255, 64, 64);
        }
    }

    for (auto i = 0; i < rows; ++i)
    {
        qDebug() << distance_matrix(i, path[i]);
        image.setPixel(i, path[i], qRgb(32, 32, 255));
    }

    ui->imageLabel->setPixmap(QPixmap::fromImage(image));
}
