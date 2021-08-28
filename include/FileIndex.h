/**************************************************************************************************
 * @file        FileIndex.h
 * @author      Thomas
 * @brief       File index for my code and functions
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * This header is used to allow for "Computed Includes" within all the code and functions that I
 * have created.
 * This will contain a list of defines, which can be changed allowing for a single point of update
 * for all of the files. This will allow for re-arranging of the folder structure without having
 * to update ALL of the files.
 *************************************************************************************************/
#ifndef FILEINDEX_H_
#define FILEINDEX_H_

/**************************************************************************************************
 * All of the defines below are for "Drives" - examples: UART interface drives, etc.
 * These would be contained within a "drv" folder
 *************************************************************************************************/
#define FilInd_GPIO___HD    "milibrary/drv/GPIO/GPIO.h"             // File for the GPIO driver
                                                                    // code
#define FilInd_DeMux__HD    "milibrary/drv/GPIO/DeMux/DeMux.h"      // File for the DeMultiplexor
                                                                    // driver
#define FilInd_StpCOREHD    "milibrary/drv/Stepper/StepperCore.h"   // File for the Stepper driver
                                                                    // Core
#define FilInd_StpDMA_HD    "milibrary/drv/Stepper/StepperDMA.h"    // DMA controlled version
#define FilInd_StpTIM_HD    "milibrary/drv/Stepper/StepperTIM.h"    // TIM controlled version

#define FilInd_USART__HD    "milibrary/drv/UARTPeriph/UARTPeriph.h" // File for the USART driver
                                                                    // code
#define FilInd_SPIPe__HD    "milibrary/drv/SPIPeriph/SPIPeriph.h"   // File for the SPI driver code
#define FilInd_I2CPe__HD    "milibrary/drv/I2CPeriph/I2CPeriph.h"   // File for the I2C driver code


#define FilInd_DMAPe__HD    "milibrary/drv/DMAPeriph/DMAPeriph.h"   // File for the DMA driver code
// DMA sub-classes
// ~~~~~~~~~~~~~~~>
#define FilIndUSARTDMAHD    "milibrary/drv/UARTPeriph/UARTDMAPeriph.h"
    // File fur the USART driver with additional support for DMA interfaces

/**************************************************************************************************
 * All of the defines below are for "Common" code - examples: Buffers, Math, etc.
 * These would be contained within a "com" folder
 *************************************************************************************************/
#define FilInd_GENBUF_TP    "milibrary/com/GenBuffer/GenBuffer.cpp" // File for the Generic Buffer
                                                                    // template

#define FilInd_DATMngrHD    "milibrary/com/DataManip/DataManip.h"   // File for Data Manipulator

/**************************************************************************************************
 * All of the defines below are for "Devices" - examples: external hardware, shift registers, etc.
 * These would be contained within a "dev" folder
 *************************************************************************************************/
#define FilInd_AS5x4x_HD    "milibrary/dev/AS5x4x/AS5x4x.h" // File for the AS5x4x Position Sensor
                                                            // devices
#define FilInd_AD741x_HD    "milibrary/dev/AD741x/AD741x.h" // File for the AD741x Temperature
                                                            // Sensor

#endif /* FILEINDEX_H_ */
