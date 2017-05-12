/********************************************************************************
** Form generated from reading UI file 'autoalignviewer.ui'
**
** Created by: Qt User Interface Compiler version 5.7.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_AUTOALIGNVIEWER_H
#define UI_AUTOALIGNVIEWER_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "pclviewer.h"

QT_BEGIN_NAMESPACE

class Ui_AutoAlignViewer
{
public:
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    PCLViewer *currentFrameViewer;
    PCLViewer *processedViewer;
    QGridLayout *gridLayout;
    QPushButton *toggleCaputureButton;
    QSpacerItem *horizontalSpacer;
    QPushButton *finalizeButton;

    void setupUi(QWidget *AutoAlignViewer)
    {
        if (AutoAlignViewer->objectName().isEmpty())
            AutoAlignViewer->setObjectName(QStringLiteral("AutoAlignViewer"));
        AutoAlignViewer->resize(717, 391);
        verticalLayout = new QVBoxLayout(AutoAlignViewer);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        currentFrameViewer = new PCLViewer(AutoAlignViewer);
        currentFrameViewer->setObjectName(QStringLiteral("currentFrameViewer"));

        horizontalLayout->addWidget(currentFrameViewer);

        processedViewer = new PCLViewer(AutoAlignViewer);
        processedViewer->setObjectName(QStringLiteral("processedViewer"));

        horizontalLayout->addWidget(processedViewer);


        verticalLayout->addLayout(horizontalLayout);

        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        toggleCaputureButton = new QPushButton(AutoAlignViewer);
        toggleCaputureButton->setObjectName(QStringLiteral("toggleCaputureButton"));
        toggleCaputureButton->setCheckable(true);

        gridLayout->addWidget(toggleCaputureButton, 0, 1, 1, 1);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer, 0, 0, 1, 1);

        finalizeButton = new QPushButton(AutoAlignViewer);
        finalizeButton->setObjectName(QStringLiteral("finalizeButton"));

        gridLayout->addWidget(finalizeButton, 0, 2, 1, 1);


        verticalLayout->addLayout(gridLayout);


        retranslateUi(AutoAlignViewer);

        QMetaObject::connectSlotsByName(AutoAlignViewer);
    } // setupUi

    void retranslateUi(QWidget *AutoAlignViewer)
    {
        AutoAlignViewer->setWindowTitle(QApplication::translate("AutoAlignViewer", "Form", Q_NULLPTR));
        toggleCaputureButton->setText(QApplication::translate("AutoAlignViewer", "Toggle Capture", Q_NULLPTR));
        finalizeButton->setText(QApplication::translate("AutoAlignViewer", "Finalize", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class AutoAlignViewer: public Ui_AutoAlignViewer {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_AUTOALIGNVIEWER_H
