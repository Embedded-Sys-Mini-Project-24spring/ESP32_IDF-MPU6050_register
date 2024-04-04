// Implementation of the Savitzky-Golay digital smoothing filter
// the implementation is a cubic polynomial with a window size of
// 5

#include <stdint.h>

// FilterData will return the filtered results of the provided
// data. The provided data must be 5 samples deep.
void FilterData( int16_t* data, uint8_t dataSize, int16_t* filteredData );