#ifndef ALIGNMENTVISUALIZATIONLIB_GLOBAL_H
#define ALIGNMENTVISUALIZATIONLIB_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(ALIGNMENTVISUALIZATIONLIB_LIBRARY)
#  define ALIGNMENTVISUALIZATIONLIBSHARED_EXPORT Q_DECL_EXPORT
#else
#  define ALIGNMENTVISUALIZATIONLIBSHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // ALIGNMENTVISUALIZATIONLIB_GLOBAL_H