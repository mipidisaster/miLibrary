/**************************************************************************************************
 * @file        DMAPeriph.cpp
 * @author      Thomas
 * @brief       Source file for the Generic DMA Class handler
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include <FileIndex.h>
#include FilInd_DMAPe__HD

void DMAPeriph::popDMARegisters(DMA_HandleTypeDef *hdma,
                                uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength) {
/**************************************************************************************************
 * Populate the Source/Destination registers of the input DMA, and the desired number of data
 * points to be transfered between the registers.
 *   This function WILL NOT enable/disable the input DMA, this needs to be done outside of the
 *   function
 *************************************************************************************************/
    /* Clear all flags */
    hdma->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << hdma->ChannelIndex);

    /* Configure DMA Channel data length */
    hdma->Instance->CNDTR = DataLength;

    /* Memory to Peripheral */
    if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
    {
      /* Configure DMA Channel destination address */
      hdma->Instance->CPAR = DstAddress;

      /* Configure DMA Channel source address */
      hdma->Instance->CMAR = SrcAddress;
    }
    /* Peripheral to Memory */
    else
    {
      /* Configure DMA Channel source address */
      hdma->Instance->CPAR = SrcAddress;

      /* Configure DMA Channel destination address */
      hdma->Instance->CMAR = DstAddress;
    }
}

DMAPeriph::DMAPeriph() {
/**************************************************************************************************
 * Construct the DMAPeriph class, no internals - only required for generic function calls
 *************************************************************************************************/
    // None

}

DMAPeriph::~DMAPeriph() {
/**************************************************************************************************
 * Destroy the DMAPeriph class, no internals
 *************************************************************************************************/
    // None
}

