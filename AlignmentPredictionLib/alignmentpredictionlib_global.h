#ifndef ALIGNMENTPREDICTIONLIB_GLOBAL_H
#define ALIGNMENTPREDICTIONLIB_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(ALIGNMENTPREDICTIONLIB_LIBRARY)
#  define ALIGNMENTPREDICTIONLIBSHARED_EXPORT Q_DECL_EXPORT
#else
#  define ALIGNMENTPREDICTIONLIBSHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // ALIGNMENTPREDICTIONLIB_GLOBAL_H
