#ifndef NUMERIC_STABILITY_H
#define NUMERIC_STABILITY_H

#include <vector>
#include <iterator>

//!
//! \brief kahan_summation
//!     A function that sums a collection of values of type DataT in a
//!     numerically stable way.
//! \param sum
//!     The sum of the collection
//! \param begin
//!     The starting location of the collection (as an Iterator)
//! \param end
//!     The ending location of the collection (as an Iterator)
//!
//!
template <class Iterator, class DataT = typename std::iterator_traits<Iterator>::value_type>
void kahan_summation(DataT &sum, Iterator begin, Iterator end) {
    DataT result = DataT{};
    DataT c = DataT{};

    for(;begin != end; ++begin)
    {
        DataT y = *begin - c;
        DataT t = result + y;
        c = (t - result) - y;
        result = t;
    }
    sum = result;
}


#endif // NUMERIC_STABILITY_H
