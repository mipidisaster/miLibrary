/**************************************************************************************************
 * @file        FileIndex.h
 * @author      Thomas
 * @version     V0.1
 * @date        1 Oct 2018
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
#define FilInd_USART__HD    "drv\UARTDevice\UARTDevice.h"   // File for the USART driver code

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


#endif /* FILEINDEX_H_ */
