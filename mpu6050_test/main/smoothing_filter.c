#include "smoothing_filter.h"
#include <stdio.h>

void FilterData( uint16_t* data, uint8_t dataSize, uint16_t* filteredData )
{
    if(dataSize != 5)
    {
        printf("Invalid number of samples attempted to be filtered.");
    }
    else
    {
        //Turn everything into fixed point values
        uint32_t tempFilteredValue = ( (-3*(((uint32_t)data[0])<<16)) + (12*(((uint32_t)data[1])<<16)) + (17*(((uint32_t)data[2])<<16)) + (12*(((uint32_t)data[3])<<16)) + (-3*(((uint32_t)data[4])<<16)) )/35;

        *filteredData = (uint16_t)(tempFilteredValue >> 16);
    }
}