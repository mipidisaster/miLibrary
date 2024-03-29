/**************************************************************************************************
 * @file        DMAPeriph.h
 * @author      Thomas
 * @brief       Header file for the Generic DMA Class handler
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * This is a very basic class, which is used to contain basic DMA functions. It is only compatible
 * with embedded devices which have DMAs within them, current supported devices:
 *      STM32
 *
 * Class includes only protected functions:
 *          ".popDMARegisters"      - Will populate the DMA registers with data source and
 *                                    destinations
 * 
 * There is no other functionality within this class.
 *************************************************************************************************/
#ifndef DMAPERIPH_H_
#define DMAPERIPH_H_

#include "FileIndex.h"
// C System Header(s)
// ------------------
#include <stdint.h>

// C++ System Header(s)
// --------------------
// None

// Other Libraries
// --------------
#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
#include "stm32f1xx_hal.h"              // Include the HAL UART library

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
#include "stm32l4xx_hal.h"              // Include the HAL UART library

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

// Defines specific within this class
// None
// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/

class DMAPeriph {
/**************************************************************************************************
 * ==   TYPES   == >>>       TYPES GENERATED WITHIN CLASS        <<<
 *   -----------
 *  Following types are generated within this class. If needed outside of the class, need to
 *  state "DMAPeriph::" followed by the type.
 *************************************************************************************************/
protected:
    enum class DMAMode : uint8_t {      // DMA modes used for UART
        kDisable    = 0x00,             // DMA is disabled
        kEnable     = 0x01              // DMA is enabled
    };

    enum DMAInterState : uint8_t {kIT_Enable, kIT_Disable};     // Enumerate state for enabling/
                                                               // disabling interrupts

/**************************************************************************************************
* == GEN PARAM == >>>       GENERIC PARAMETERS FOR CLASS        <<<
*   -----------
*  Parameters required for the class to function.
*************************************************************************************************/
// None

/**************************************************************************************************
 * == SPC PARAM == >>>        SPECIFIC ENTRIES FOR CLASS         <<<
 *   -----------
 *  Following are functions and parameters which are specific for the embedded device selected.
 *  The initialisation function for the class is also within this section, which again will be
 *  different depending upon the embedded device selected.
 *************************************************************************************************/
public:
    DMAPeriph();        // Construct class - No internals

    virtual ~DMAPeriph();

/**************************************************************************************************
 * == GEN FUNCT == >>>      GENERIC FUNCTIONS WITHIN CLASS       <<<
 *   -----------
 *  The following are functions scoped within the "DMAPeriph" class, which are generic; this
 *  means are used by ANY of the embedded devices supported by this class.
 *  The internals of the class, will then determine how it will be managed between the multiple
 *  embedded devices.
 *************************************************************************************************/
protected:
    static void enableDMA(DMA_HandleTypeDef *hdma);
    static void disableDMA(DMA_HandleTypeDef *hdma);

    // Configure the Interrupt enabling registers/bits
    void configDMAErrorIT(DMA_HandleTypeDef *hdma, DMAInterState intr);
    void configDMAHalfTransmtIT(DMA_HandleTypeDef *hdma, DMAInterState intr);
    void configDMACmpltTransmtIT(DMA_HandleTypeDef *hdma, DMAInterState intr);

    uint8_t halfDMATransmitITChk(DMA_HandleTypeDef *hdma);  // Check to see if interrupt is enabled
    uint8_t comptDMATransmitITChk(DMA_HandleTypeDef *hdma); // Check to see if interrupt is enabled
    uint8_t transmitDMAErrorITChk(DMA_HandleTypeDef *hdma); // Check to see if interrupt is enabled

    void clearGlobalDMAFlg(DMA_HandleTypeDef *hdma);        // Clear Interrupt flags
    void clearHalfDMATransmitFlg(DMA_HandleTypeDef *hdma);  // Clear Interrupt flags
    void clearComptDMATransmitFlg(DMA_HandleTypeDef *hdma); // Clear Interrupt flags
    void clearTransmitDMAErrorFlg(DMA_HandleTypeDef *hdma); // Clear Interrupt flags

    uint8_t globalDMAChk(DMA_HandleTypeDef *hdma);          // Check state of the global interrupt
                                                            // flag
    uint8_t halfDMATransmitChk(DMA_HandleTypeDef *hdma);    // Check state of the half transmit
                                                            // interrupt flag
    uint8_t comptDMATransmitChk(DMA_HandleTypeDef *hdma);   // Check state of the transmit
                                                            // complete interrupt flag
    uint8_t transmitDMAErrorChk(DMA_HandleTypeDef *hdma);   // Check state of the transmit error
                                                            // flag

    static void popDMARegisters(DMA_HandleTypeDef *hdma,
                                uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
        // Update the DMA registers with the required data, so that it can be enabled
        //      Enabling/Disabling not covered by this function
};

#endif /* DMAPERIPH_H_ */
