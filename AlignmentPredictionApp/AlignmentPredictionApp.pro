QT += core gui widgets

CONFIG += c++11

TARGET = AlignmentPredictionApp
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

include(../pcl.pri)

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../AlignmentPredictionLib/release/ -lAlignmentPredictionLib
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../AlignmentPredictionLib/debug/ -lAlignmentPredictionLib
else:unix: LIBS += -L$$OUT_PWD/../AlignmentPredictionLib/ -lAlignmentPredictionLib

INCLUDEPATH += $$PWD/../AlignmentPredictionLib
DEPENDPATH += $$PWD/../AlignmentPredictionLib

FORMS += pclviewer.ui \
    autoalignviewer.ui \
    distancematrixvisualizer.ui

HEADERS += pclviewer.h \
    openni2handler.h \
    autoalignviewer.h \
    autoaligner.h \
    distancematrixvisualizer.h \
    pointcloudnormalhandler.h

SOURCES += main.cpp \
pclviewer.cpp \
    openni2handler.cpp \
    autoalignviewer.cpp \
    autoaligner.cpp \
    distancematrixvisualizer.cpp

RESOURCES += \
    ../icons/icons.qrc
