#include "multivariate_statistics.h"

void calculate_entropy(double &entropy, double p)
{
    entropy = -p * std::log2(p) - (1 - p) * std::log2(1 - p);
}
