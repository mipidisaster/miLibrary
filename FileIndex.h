/**************************************************************************************************
 * @file        FileIndex.h
 * @author      Thomas
 * @version     V0.2
 * @date        07 Oct 2018
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
 * for all of the files. This will allow for re-arranging of the folder sturcture without having
 * to update ALL of the files.
 *************************************************************************************************/
#ifndef FILEINDEX_H_
#define FILEINDEX_H_

/**************************************************************************************************
 * All of the defines below are for "Drives" - examples: UART interface drives, etc.
 * These would be contained within a "drv" folder
 *************************************************************************************************/
#define FilInd_GPIO___HD    "drv\GPIO\GPIO.h"               // File for the GPIO driver code
#define FilInd_USART__HD    "drv\UARTDevice\UARTDevice.h"   // File for the USART driver code
#define FilInd_SPIDe__HD    "drv\SPIDevice\SPIDevice.h"     // File for the SPI driver code
#define FilInd_I2CDe__HD    "drv\I2CDevice\I2CDevice.h"     // File for the I2C driver code

/**************************************************************************************************
 * All of the defines below are for "Common" code - examples: Buffers, Math, etc.
 * These would be contained within a "com" folder
 *************************************************************************************************/
#define FilInd_GENBUF_HD    "com\GenBuffer\GenBuffer.h"     // File for the Generic Buffer code
#define FilInd_GENBUF_Cp    "com\GenBuffer\GenBuffer.cpp"   // File for the Generic Buffer code

/**************************************************************************************************
 * All of the defines below are for "Devices" - examples: external hardware, shift registers, etc.
 * These would be contained within a "dev" folder
 *************************************************************************************************/
#define FilInd_DeMux__HD    "dev\DeMux\DeMux.h"             // File for the DeMultiplexor device
#define FilInd_AS5x4__HD    "dev\AS5x4x\AS5x4x.h"           // File for the AS5x4x Position Sensor
                                                            // devices

#endif /* FILEINDEX_H_ */
