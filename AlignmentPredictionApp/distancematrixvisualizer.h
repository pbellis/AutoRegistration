#ifndef DISTANCEMATRIXVISUALIZER_H
#define DISTANCEMATRIXVISUALIZER_H

#include <QWidget>
#include <Eigen/Dense>

namespace Ui {
class DistanceMatrixVisualizer;
}

class DistanceMatrixVisualizer : public QWidget
{
    Q_OBJECT

public:
    explicit DistanceMatrixVisualizer(QWidget *parent = 0);
    ~DistanceMatrixVisualizer();

public slots:
    void onRecieveAlignmentStrategy(const Eigen::MatrixXd &distance_matrix, const Eigen::VectorXi &path);

private:
    Ui::DistanceMatrixVisualizer *ui;
};

#endif // DISTANCEMATRIXVISUALIZER_H
