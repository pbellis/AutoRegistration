#include "clustering.h"

#include <numeric>
#include <algorithm>

void calculate_optimal_split(double &split, const std::vector<double> &data, double min_value, double max_value, bool sorted)
{
    const size_t N = data.size();

    if (sorted)
    {
        double max_gap = 0;
        for (size_t i = 1; i < N; ++i)
        {
            if (data[i - 1] < min_value || data[i] > max_value) continue;
            double entropy;
            calculate_entropy(entropy, data[i] / 2);
            const double gap = entropy * (data[i] - data[i - 1]);
            if (max_gap < gap)
            {
                max_gap = gap;
                split = data[i];
            }
        }
    }
    else
    {
        std::vector<double> sorted_data(N);
        std::copy(data.cbegin(), data.cend(), sorted_data.begin());
        std::sort(sorted_data.begin(), sorted_data.end());

        double max_gap = 0;
        for (size_t i = 1; i < N; ++i)
        {
            if (sorted_data[i - 1] < min_value || sorted_data[i] > max_value) continue;
            double entropy;
            calculate_entropy(entropy, data[i] / 2);
            const double gap = entropy * (sorted_data[i] - sorted_data[i - 1]);
            if (max_gap < gap)
            {
                max_gap = gap;
                split = sorted_data[i];
            }
        }
    }
}
