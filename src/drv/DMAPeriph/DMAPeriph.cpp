/**************************************************************************************************
 * @file        DMAPeriph.cpp
 * @author      Thomas
 * @brief       Source file for the Generic DMA Class handler
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include <FileIndex.h>                  // Header for miLibrary index
#include FilInd_DMAPe__HD               // Header for DMA

// C System Header(s)
// ------------------
#include <stdint.h>

// C++ System Header(s)
// --------------------

// Other Libraries
// --------------
#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
#include "stm32f1xx_hal.h"              // Include the HAL DMA library

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
#include "stm32l4xx_hal.h"              // Include the HAL DMA library

#elif (zz__MiEmbedType__zz == 10)       // If the target device is an Raspberry Pi then
//=================================================================================================
#error "Unsupported target device"

#elif (defined(zz__MiEmbedType__zz)) && (zz__MiEmbedType__zz ==  0)
//     If using the Linux (No Hardware) version then
//=================================================================================================
#error "Unsupported target device"

#else
//=================================================================================================
#error "Unrecognised target device"

#endif

// Project Libraries
// -----------------
#include FilInd_GENBUF_TP               // Provide the template for the circular buffer class

//=================================================================================================

void DMAPeriph::enableDMA(DMA_HandleTypeDef *hdma) {
/**************************************************************************************************
 * Enable the input DMA interface
 *************************************************************************************************/
    __HAL_DMA_ENABLE(hdma);
}

void DMAPeriph::disableDMA(DMA_HandleTypeDef *hdma) {
/**************************************************************************************************
 * Disable the input DMA interface
 *************************************************************************************************/
    __HAL_DMA_DISABLE(hdma);
}


void DMAPeriph::configDMAErrorIT(DMA_HandleTypeDef *hdma, DMAInterState intr) {
/**************************************************************************************************
 * Configure the DMA interrupt bit for Transmit Errors
 *************************************************************************************************/
    if (intr == DMAInterState::kIT_Enable) {                // If request is to enable
        __HAL_DMA_ENABLE_IT(hdma, DMA_IT_TE);               // Enable interrupt
    }
    else {                                                  // If request is to disable
        __HAL_DMA_DISABLE_IT(hdma, DMA_IT_TE);              // Then disable interrupt
    }
}

void DMAPeriph::configDMAHalfTransmtIT(DMA_HandleTypeDef *hdma, DMAInterState intr) {
/**************************************************************************************************
 * Configure the DMA interrupt bit for Half Transmit complete
 *************************************************************************************************/
    if (intr == DMAInterState::kIT_Enable) {                // If request is to enable
        __HAL_DMA_ENABLE_IT(hdma, DMA_IT_HT);               // Enable interrupt
    }
    else {                                                  // If request is to disable
        __HAL_DMA_DISABLE_IT(hdma, DMA_IT_HT);              // Then disable interrupt
    }
}

void DMAPeriph::configDMACmpltTransmtIT(DMA_HandleTypeDef *hdma, DMAInterState intr) {
/**************************************************************************************************
 * Configure the DMA interrupt bit for Transmit complete
 *************************************************************************************************/
    if (intr == DMAInterState::kIT_Enable) {                // If request is to enable
        __HAL_DMA_ENABLE_IT(hdma, DMA_IT_TC);               // Enable interrupt
    }
    else {                                                  // If request is to disable
        __HAL_DMA_DISABLE_IT(hdma, DMA_IT_TC);              // Then disable interrupt
    }
}

uint8_t DMAPeriph::halfDMATransmitITChk(DMA_HandleTypeDef *hdma) {
/**************************************************************************************************
 * Check to see whether the Transmit half complete interrupt has been enabled
 *************************************************************************************************/
    if ( __HAL_DMA_GET_IT_SOURCE(hdma, DMA_IT_HT) != 0 )
        return (1);
    else
        return (0);
}

uint8_t DMAPeriph::comptDMATransmitITChk(DMA_HandleTypeDef *hdma) {
/**************************************************************************************************
 * Check to see whether the Transmit complete interrupt has been enabled
 *************************************************************************************************/
    if ( __HAL_DMA_GET_IT_SOURCE(hdma, DMA_IT_TC) != 0 )
        return (1);
    else
        return (0);
}
uint8_t DMAPeriph::transmitDMAErrorITChk(DMA_HandleTypeDef *hdma) {
/**************************************************************************************************
 * Check to see whether the Transmit complete interrupt has been enabled
 *************************************************************************************************/
    if ( __HAL_DMA_GET_IT_SOURCE(hdma, DMA_IT_TE) != 0 )
        return (1);
    else
        return (0);
}

void DMAPeriph::clearGlobalDMAFlg(DMA_HandleTypeDef *hdma) {
/**************************************************************************************************
 * Clear the Interrupt register for hdma - Global Interrupt flag
 *************************************************************************************************/
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_GI_FLAG_INDEX(hdma));
}

void DMAPeriph::clearHalfDMATransmitFlg(DMA_HandleTypeDef *hdma) {
/**************************************************************************************************
 * Clear the Interrupt register for hdma - Transmit Half complete Interrupt flag
 *************************************************************************************************/
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_HT_FLAG_INDEX(hdma));
}

void DMAPeriph::clearComptDMATransmitFlg(DMA_HandleTypeDef *hdma) {
/**************************************************************************************************
 * Clear the Interrupt register for hdma - Transmit complete Interrupt flag
 *************************************************************************************************/
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));
}

void DMAPeriph::clearTransmitDMAErrorFlg(DMA_HandleTypeDef *hdma) {
/**************************************************************************************************
 * Clear the Interrupt register for hdma - Transmit error Interrupt flag
 *************************************************************************************************/
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TE_FLAG_INDEX(hdma));
}

uint8_t DMAPeriph::globalDMAChk(DMA_HandleTypeDef *hdma) {
/**************************************************************************************************
 * Check the status of DMA global interrupt flag is set
 *************************************************************************************************/
    if ( __HAL_DMA_GET_FLAG(hdma, __HAL_DMA_GET_GI_FLAG_INDEX(hdma)) != 0 )
        return (1);
    else
        return (0);
}

uint8_t DMAPeriph::halfDMATransmitChk(DMA_HandleTypeDef *hdma) {
/**************************************************************************************************
 * Check the status of DMA half transmit interrupt flag is set
 *************************************************************************************************/
    if ( __HAL_DMA_GET_FLAG(hdma, __HAL_DMA_GET_HT_FLAG_INDEX(hdma)) != 0 )
        return (1);
    else
        return (0);
}

uint8_t DMAPeriph::comptDMATransmitChk(DMA_HandleTypeDef *hdma) {
/**************************************************************************************************
 * Check the status of DMA transmit complete interrupt flag is set
 *************************************************************************************************/
    if ( __HAL_DMA_GET_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma)) != 0 )
        return (1);
    else
        return (0);
}

uint8_t DMAPeriph::transmitDMAErrorChk(DMA_HandleTypeDef *hdma) {
/**************************************************************************************************
 * Check the status of DMA transmit error interrupt flag is set
 *************************************************************************************************/
    if ( __HAL_DMA_GET_FLAG(hdma, __HAL_DMA_GET_TE_FLAG_INDEX(hdma)) != 0 )
        return (1);
    else
        return (0);
}

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

