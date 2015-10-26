

#define VREF				( 2.5f )
#define BITS				( 4096 )

/**
 *  Gets a raw, filtered reading from the ADC.
 * @param channelIndex - Index of the channel to read.
 * @param filter - Array of FIR filter coefficients.
 * @param filterLength - Length of the filter in the array.
 * @return ADC reading.
 */
q15_t adc_get_reading_raw ( ADC_Channel_t channelIndex, q15_t* filter, uint16_t filterLength )
{
    int32_t result = 0;

    // Determine which DMA "slot" is the latest to be filled.  It is located two behind
    // the current target.  The current target points to the next slot to be filled.  The
    // one previous is currently being filled.  The one before that is the most recent
    // complete value.

    int16_t targetSlot = (dmaTargetIndex - 2);
    if (targetSlot < 0) { targetSlot += DMA_DEPTH; }

    int16_t offset = targetSlot * ADC_NUM_CHANNELS + channelIndex;

    if (filterLength > 0)
    {
        for (int i = 0; i < filterLength; i++)
        {
            result += (int32_t) adcDMA[offset] * (int32_t) filter[i] ;

            offset -= ADC_NUM_CHANNELS;

            // Wrap around correctly.
            if (offset < 0) { offset += DMA_DEPTH * ADC_NUM_CHANNELS; }
        }

        return (q15_t) (result >> 15);
    }
    else
    {
        return adcDMA[offset];
    }
}