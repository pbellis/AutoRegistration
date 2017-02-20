#-------------------------------------------------
#
# Project created by QtCreator 2017-01-06T13:14:54
#
#-------------------------------------------------

QT       -= gui

TARGET = AlignmentPredictionLib
TEMPLATE = lib

DEFINES += ALIGNMENTPREDICTIONLIB_LIBRARY

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

include(../pcl.pri)
include(../boost.pri)

SOURCES += \
    prediction.cpp \
    graph_theory.cpp \
    clustering.cpp \
    statistical_shape.cpp \
    statistical_distance.cpp \
    multivariate_statistics.cpp

HEADERS +=\
        alignmentpredictionlib_global.h \
    prediction.h \
    graph_theory.h \
    clustering.h \
    multivariate_statistics.h \
    statistical_shape.h \
    statistical_distance.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}
