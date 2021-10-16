/**************************************************************************************************
 * @file        DataManip.h
 * @author      Thomas
 * @brief       Header file for the universal data manipulator functions
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * Set of functions allows for conversion from 16bits to 32bits variables to 8bit versions. This
 * is to allow for transmission down 8bit buses.
 *************************************************************************************************/
#ifndef DATAMANIP_H_
#define DATAMANIP_H_

#include "FileIndex.h"
// C System Header(s)
// ------------------
#include <stdint.h>

// C++ System Header(s)
// --------------------
// None

// Other Libraries
// --------------
#if   (zz__MiEmbedType__zz ==  50)      // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
// None

#elif (zz__MiEmbedType__zz ==  51)      // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
// None

#elif (zz__MiEmbedType__zz ==  10)      // If the target device is an Raspberry Pi then
//=================================================================================================
// None

#elif (defined(zz__MiEmbedType__zz)) && (zz__MiEmbedType__zz ==  0)
//     If using the Linux (No Hardware) version then
//=================================================================================================
// None

#else
//=================================================================================================
#error "Unrecognised target device"

#endif

// Project Libraries
// -----------------
// None

//=================================================================================================

// Defines specific within this class
// None
// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/

namespace DataManip {
    /**********************************************************************************************
     * Function set is able to convert from unsigned 16bit, to 2 bytes
     *********************************************************************************************/
    void        _16bit_2_2x8bit(uint16_t sourceData,  uint8_t  *arrayData);
    uint16_t    _2x8bit_2_16bit(uint8_t *arrayData);

    /**********************************************************************************************
     * Function set is able to convert from unsigned 32bit, to 4 bytes
     *********************************************************************************************/
    void        _32bit_2_4x8bit(uint32_t sourceData,  uint8_t *arrayData);
    uint32_t    _4x8bit_2_32bit(uint8_t *arrayData);

    /**********************************************************************************************
     * Function set is able to convert from float, to 4S bytes
     *********************************************************************************************/
    void        _float_2_4x8bit(float sourceData,     uint8_t *arrayData);
    float       _4x8bit_2_float(uint8_t *arrayData);
}

#endif /* DATAMANIP_H_ */
