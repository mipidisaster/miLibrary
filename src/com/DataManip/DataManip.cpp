/**************************************************************************************************
 * @file        DataManip.cpp
 * @author      Thomas
 * @version     V0.2
 * @date        22 Jun 2019
 * @brief       Source file for the universal data manipulator functions
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
#include <FileIndex.h>
#include FilInd_DATMngrHD

using namespace DataManip;

void      DataManip::_16bit_2_2x8bit(uint16_t sourceData,  uint8_t  *arrayData) {
/**************************************************************************************************
 * Function will take the input 16bit value, and convert to a 8bit array of two elements.
 * Where the first entry will contain the Most Significant Byte (MSB) - Big-endian!
 *************************************************************************************************/
    uint16_t temp   = 0xFF00;           // Initialise temporary variable with all upper 8bits set
    uint8_t  i = 0;                     // Variable used for looping
    uint8_t  shiftval = 0;              // Variable used to store binary shift amount

    for (i = 0; i != 2; i++)            // Cycle through array
    {
        shiftval = ( 1 - i ) * 8;                           // Calculate required shift amount
        arrayData[i] = (( temp & sourceData) >> shiftval);  // AND together the temp and input
                                                            // 'sourceData', and shift by
                                                            // calculated amount
        temp = temp >> 8;       // Shift down the temp array by 1 byte (8 bits) so that mask
                                // now captures the next part of the data
    }
}

uint16_t  DataManip::_2x8bit_2_16bit(uint8_t *arrayData) {
/**************************************************************************************************
 * Function will take the input array data, and convert to a 16bit (unsigned) value.
 * Note the function is expecting that the first array element will contain the Most Significant
 * Byte (MSB) of the expected output - Big-endian!
 *************************************************************************************************/
    uint16_t temp = 0;                  // Temporary variable to contain the final output
    uint8_t  i = 0;                     // Variable used for looping
    uint8_t  shiftval = 0;              // Variable used to store binary shift amount

    for (i = 0; i != 2; i++)            // Cycle through array
    {
        shiftval = ( 1 - i ) * 8;                           // Calculate required shift amount
        temp |= (  (uint16_t)(arrayData[i]) << shiftval  ); // Take array data, and shift up to
                                                            // correct position in output type
    }

    return( temp );         // Return converted variable
}

void      DataManip::_32bit_2_4x8bit(uint32_t sourceData,  uint8_t  *arrayData) {
/**************************************************************************************************
 * Function will take the input 32bit value, and convert to a 8bit array of four elements.
 * Where the first entry will contain the Most Significant Byte (MSB) - Big-endian!
 *************************************************************************************************/
    uint32_t temp   = 0xFF000000;       // Initialise temporary variable with all upper 8bits set
    uint8_t  i = 0;                     // Variable used for looping
    uint8_t  shiftval = 0;              // Variable used to store binary shift amount

    for (i = 0; i != 4; i++)            // Cycle through array
    {
        shiftval = ( 3 - i ) * 8;                           // Calculate required shift amount
        arrayData[i] = (( temp & sourceData) >> shiftval);  // AND together the temp and input
                                                            // 'sourceData', and shift by
                                                            // calculated amount
        temp = temp >> 8;       // Shift down the temp array by 1 byte (8 bits) so that mask
                                // now captures the next part of the data
    }
}

uint32_t  DataManip::_4x8bit_2_32bit(uint8_t *arrayData) {
/**************************************************************************************************
 * Function will take the input array data, and convert to a 32bit (unsigned) value.
 * Note the function is expecting that the first array element will contain the Most Significant
 * Byte (MSB) of the expected output - Big-endian!
 *************************************************************************************************/
    uint32_t temp = 0;                  // Temporary variable to contain the final output
    uint8_t  i = 0;                     // Variable used for looping
    uint8_t  shiftval = 0;              // Variable used to store binary shift amount

    for (i = 0; i != 4; i++)            // Cycle through array
    {
        shiftval = ( 3 - i ) * 8;                           // Calculate required shift amount
        temp |= (  (uint32_t)(arrayData[i]) << shiftval  ); // Take array data, and shift up to
                                                            // correct position in output type
    }

    return( temp );         // Return converted variable
}

void      DataManip::_float_2_4x8bit(float sourceData,     uint8_t *arrayData) {
/**************************************************************************************************
 * Function will take the input float value, and convert to a 8bit array of four elements.
 * Where the first entry will contain the Most Significant Byte (MSB) - Big-endian!
 *************************************************************************************************/
    _32bit_2_4x8bit(  *(uint32_t *) &sourceData,  arrayData  );
        // Use already created function to convert unsigned 32bit to array data
}

float     DataManip::_4x8bit_2_float(uint8_t *arrayData) {
/**************************************************************************************************
 * Function will take the input array data, and convert to a float value.
 * Note the function is expecting that the first array element will contain the Most Significant
 * Byte (MSB) of the expected output - Big-endian!
 *************************************************************************************************/
    uint32_t temp = _4x8bit_2_32bit(arrayData);     // Take data, and convert to unsigned 32 bit

    return(  *(float *) &temp  );                   // Return data after typecasting to float
}
