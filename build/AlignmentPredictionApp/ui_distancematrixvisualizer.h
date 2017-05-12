/********************************************************************************
** Form generated from reading UI file 'distancematrixvisualizer.ui'
**
** Created by: Qt User Interface Compiler version 5.7.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DISTANCEMATRIXVISUALIZER_H
#define UI_DISTANCEMATRIXVISUALIZER_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_DistanceMatrixVisualizer
{
public:
    QVBoxLayout *verticalLayout;
    QLabel *imageLabel;

    void setupUi(QWidget *DistanceMatrixVisualizer)
    {
        if (DistanceMatrixVisualizer->objectName().isEmpty())
            DistanceMatrixVisualizer->setObjectName(QStringLiteral("DistanceMatrixVisualizer"));
        DistanceMatrixVisualizer->resize(534, 534);
        verticalLayout = new QVBoxLayout(DistanceMatrixVisualizer);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        imageLabel = new QLabel(DistanceMatrixVisualizer);
        imageLabel->setObjectName(QStringLiteral("imageLabel"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(imageLabel->sizePolicy().hasHeightForWidth());
        imageLabel->setSizePolicy(sizePolicy);
        imageLabel->setMinimumSize(QSize(512, 512));
        imageLabel->setScaledContents(true);

        verticalLayout->addWidget(imageLabel);


        retranslateUi(DistanceMatrixVisualizer);

        QMetaObject::connectSlotsByName(DistanceMatrixVisualizer);
    } // setupUi

    void retranslateUi(QWidget *DistanceMatrixVisualizer)
    {
        DistanceMatrixVisualizer->setWindowTitle(QApplication::translate("DistanceMatrixVisualizer", "Form", Q_NULLPTR));
        imageLabel->setText(QApplication::translate("DistanceMatrixVisualizer", "TextLabel", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class DistanceMatrixVisualizer: public Ui_DistanceMatrixVisualizer {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DISTANCEMATRIXVISUALIZER_H
