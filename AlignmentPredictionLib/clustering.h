#ifndef CLUSTERING_H
#define CLUSTERING_H

#include <vector>
#include "multivariate_statistics.h"

#include "alignmentpredictionlib_global.h"

// NOT USED AND DOESN'T HAVE A PURPOSE CURRENTLY
void ALIGNMENTPREDICTIONLIBSHARED_EXPORT calculate_optimal_split(double &split, const std::vector<double> &data, double min_value, double max_value, bool sorted = false);

#endif // CLUSTERING_H
