TEMPLATE = subdirs
CONFIG += ordered

SUBDIRS += \
    AlignmentPredictionLib \
    AlignmentPredictionApp

AlignmentPredictionApp.depends += AlignmentPredictionLib
