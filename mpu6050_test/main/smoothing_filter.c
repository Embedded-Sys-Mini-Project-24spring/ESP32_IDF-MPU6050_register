#include "smoothing_filter.h"
#include <stdio.h>

void FilterData( int16_t* data, uint8_t dataSize, int16_t* filteredData )
{
    if(dataSize != 5)
    {
        printf("Invalid number of samples attempted to be filtered.");
    }
    else
    {
        //Turn everything into fixed point values
        int32_t tempFilteredValue = ( (-3*(((int32_t)data[0])<<16)) + (12*(((int32_t)data[1])<<16)) + (17*(((int32_t)data[2])<<16)) + (12*(((int32_t)data[3])<<16)) + (-3*(((int32_t)data[4])<<16)) )/35;

        *filteredData = (int16_t)(tempFilteredValue >> 16);
    }
}