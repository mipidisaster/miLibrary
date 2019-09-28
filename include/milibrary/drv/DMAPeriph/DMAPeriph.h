/**************************************************************************************************
 * @file        DMAPeriph.h
 * @author      Thomas
 * @version     V0.1
 * @date        28 Sep 2019
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
#include <stdint.h>

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
#include "stm32f1xx_hal.h"              // Include the HAL library

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================
#include "stm32l4xx_hal.h"              // Include the HAL library

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
#error "Unsupported target device"

#else
//==================================================================================================
#error "Unrecognised target device"

#endif

// Defines specific within this class
// None

// Types used within this class
// Defined within the class, to ensure are contained within the correct scope

class DMAPeriph {
/**************************************************************************************************
 * ==   TYPES   == >>>       TYPES GENERATED WITHIN CLASS        <<<
 *   -----------
 *  Following types are generated within this class. If needed outside of the class, need to
 *  state "DMAPeriph::" followed by the type.
 *************************************************************************************************/
protected:
    enum class DMAMode : uint8_t {      // DMA modes used for UART
        disable     = 0x00,             // DMA is disabled
        enable      = 0x01              // DMA is enabled
    };

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

/**************************************************************************************************
 * == GEN FUNCT == >>>      GENERIC FUNCTIONS WITHIN CLASS       <<<
 *   -----------
 *  The following are functions scoped within the "DMAPeriph" class, which are generic; this
 *  means are used by ANY of the embedded devices supported by this class.
 *  The internals of the class, will then determine how it will be managed between the multiple
 *  embedded devices.
 *************************************************************************************************/
protected:
    static void popDMARegisters(DMA_HandleTypeDef *hdma,
                                uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
        // Update the DMA registers with the required data, so that it can be enabled
        //      Enabling/Disabling not covered by this function

    virtual ~DMAPeriph();
};

#endif /* DMAPERIPH_H_ */
