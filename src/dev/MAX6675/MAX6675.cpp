/**************************************************************************************************
 * @file        MAX6675.cpp
 * @author      Thomas
 * @brief       Source file for the MAX6675 Thermocouple Digital Converter
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include "FileIndex.h"
#include FilInd_MAX6675HD

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
// None

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
// None

#elif (zz__MiEmbedType__zz == 10)       // If the target device is an Raspberry Pi then
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
#include FilInd_GPIO___HD               // Allow use of GPIO class, for Chip Select
#include FilInd_DeMux__HD               // Allow use of DeMux class, for Chip Select
#include FilInd_SPIPe__HD               // Include class for SPI Peripheral

//=================================================================================================

void MAX6675::popGenParam(void) {
/**************************************************************************************************
 * Generate default parameters for the MAX6675 device. To be called by all constructors.
 * Initial construction will populate the internal GenBuffers with default parameters (as basic
 * constructor of GenBuffer is already set to zero).
 *************************************************************************************************/
    _gpio_select_           = __null;       // All parameters set to zero/null
    _demux_select_          = __null;       //
    _demux_select_number_   = -1;           //
    temp                    = -999;         // Default Temperature to -999Degrees

    raw_data                = 0;            // Initialise to zero
    _device_contents_[0]    = 0xFF;         //
    _device_contents_[1]    = 0xFF;         //

    flt                     = DevFlt::kInitialised; //

    spi_read_flt      = SPIPeriph::DevFlt::kNone;   // SPI read fault status set to "None"
    read_cmp_flag     = 0x00;                       // Initialise the communication complete flag
    read_cmp_target   = 0x00;                       // Initialise the target communication count
}

MAX6675::MAX6675(void) {
/**************************************************************************************************
 * Basic construction of the MAX6675 device, this assumes that the hardwared SPI chip select is
 * being used, and therefore not individual GPIO, or DeMux interface needs to be setup.
 *************************************************************************************************/
    popGenParam();          // Call default setup, to populate generic entries
}

MAX6675::MAX6675(GPIO *ChipSelect) {
/**************************************************************************************************
 * OVERLOADED function with specific GPIO pin for the MAX6675 device.
 * Will override the internal parameter '_gpio_select_' to point to this GPIO signal.
 *************************************************************************************************/
    popGenParam();          // Call default setup, to populate generic entries

    _gpio_select_           = ChipSelect;   // Copy across the GPIO Chip Select pointer
}

MAX6675::MAX6675(DeMux *DeMuxCS, uint8_t CSNum) {
/**************************************************************************************************
 * OVERLOADED function with specific GPIO pin for the MAX6675 device.
 * Will override the internal parameter '_demux_select_' and '_demux_select_number_' to point to
 * input parameters.
 *************************************************************************************************/
    popGenParam();          // Call default setup, to populate generic entries

    _demux_select_          = DeMuxCS;      // Copy across the DeMux pointer
    _demux_select_number_   = CSNum;        // Store the DeMux selection number for MAX6675
}

MAX6675::DevFlt MAX6675::decodeMAX6675(void) {
/**************************************************************************************************
 * After the internal register of class has been populated '_device_contents_', this function
 * will decode the data, and determine if the device has provided invalid data, or what the
 * temperature read is.
 *************************************************************************************************/
    uint16_t    register_data = 0;      // Data to store the MAX6675 register

    register_data = ((uint16_t)_device_contents_[0] << 8) | _device_contents_[1];
    // Concatenate the internal array together into a 16bit variable (original contents from
    // device)
    raw_data    = register_data;        // Copy contents into class variable

    if ((register_data & (MAX6675_Dummy | MAX6675_NoThermo | MAX6675_DevID)) != 0x0000) {
        // If the data read contains an incorrect value, then determine fault, and setup fault flag
        if ((register_data & MAX6675_NoThermo) != 0x0000) {     // If open circuit on sensor
            flt     = DevFlt::kNo_Sensor;                       // Set no sensor fault
            return (flt);                                       // Return fault state
        }
        else if ((register_data & MAX6675_Dummy) != 0x0000) {   // If Dummy bit is incorrect
            flt     = DevFlt::kDummy_Fault;                     // Set Dummy fault
            return (flt);                                       // Return fault state
        }
        else {                                                  // If Device ID is incorrect
            flt     = DevFlt::kDevice_ID_Fault;                 // Set Device ID fault
            return (flt);                                       // Return fault state
        }
    }
    // If have reached this point, then there is no recognised fault with the read/data
    register_data   &= MAX6675_Temp;    // Retain only the 12bits for temperature
    register_data   >>= 3;              // Shift down by 3 bits
    temp            = ((float)register_data) * 0.25;
    // Resolution of data is 1 bit = 0.25Degrees, therefore multiple by 0.25 and convert to
    // float type (single precision)

    return (flt = DevFlt::kNone);       // Return that the device has provided valid data
}

void MAX6675::poleTempRead(uint8_t *data_registers) {
/**************************************************************************************************
 * Function will be provided with an array of data points (expects only 2), which will be read
 * into the class.
 * For the function 'devodeMAX6675' to decode the data and retrieve the temperature
 *************************************************************************************************/
    _device_contents_[0] = data_registers[0];
    _device_contents_[1] = data_registers[1];

    decodeMAX6675();
}

void MAX6675::poleTempRead(SPIPeriph *Interface) {
/**************************************************************************************************
 * Function will send a read request to "this" MAX6675 device, overriding the contents of internal
 * register '_device_contents_', with the latest data of device.
 * Any faults with the transfer or incorrect register states will cause fault flag to be
 * invalidated.
 *
 * Depending upon how the MAX6675 was initialised, it will call different variations of the
 * SPIDevice - SPITransfer, either normally (hardware chip select), or with the linked GPIO
 * (software chip select)
 *************************************************************************************************/
    SPIPeriph::DevFlt read_flt = SPIPeriph::DevFlt::kNone;

    if      ( _gpio_select_ != __null ) {       // If GPIO has been provided
        read_flt = Interface->poleMasterTransfer(_gpio_select_,
                                                 &_device_contents_[0],
                                                 &_device_contents_[0], 2);
    }

    else if (_demux_select_ != __null) {        // If DeMux has been provided
        read_flt = Interface->poleMasterTransfer(_demux_select_, _demux_select_number_,
                                                 &_device_contents_[0],
                                                 &_device_contents_[0], 2);
    }

    else {
        read_flt = Interface->poleMasterTransfer(&_device_contents_[0],
                                                 &_device_contents_[0], 2);
    }

    if (read_flt != SPIPeriph::DevFlt::kNone) { // If there was a read fault
        flt = DevFlt::kRead_Error;              // Capture within fault flag, and DO NOT UPDATE
    }
    else {              // With good data, decode contents
        decodeMAX6675();
    }
}

void MAX6675::reInitialise(void) {
/**************************************************************************************************
 * This function is to be called when the internal mechanics of the class need to be reset. This
 * is likely to occur in fault situations, or at initialisation.
 *************************************************************************************************/
    flt     = DevFlt::kInitialised;     // Set fault to initialised
}

void MAX6675::intTempRead(SPIPeriph *Interface) {
/**************************************************************************************************
 * Function will send a read request to "this" MAX6675 device, overriding the contents of internal
 * register '_device_contents_', with the latest data of device.
 *  (INTERRUPT BASED REQUEST)
 *
 * Depending upon how the MAX6675 was initialised, it will call different variations of the
 * SPIDevice - SPITransfer, either normally (hardware chip select), or with the linked GPIO
 * (software chip select)
 *************************************************************************************************/
    if      ( _gpio_select_ != __null ) {       // If GPIO has been provided
        Interface->intMasterTransfer(_gpio_select_, 2,
                                     &_device_contents_[0],
                                     &_device_contents_[0],
                                     &spi_read_flt, &read_cmp_flag);
    }

    else if (_demux_select_ != __null) {        // If DeMux has been provided
        // NOT SUPPORTED YET! - See issue #12 in github
    }

    else {
        Interface->intMasterTransfer(2,
                                     &_device_contents_[0],
                                     &_device_contents_[0],
                                     &spi_read_flt, &read_cmp_flag);
    }

    read_cmp_target     += 2;
}

void MAX6675::intCheckCommStatus(void) {
/**************************************************************************************************
 * Will first check to see if all the requested data events have been processed, i.e.
 * 'read_cmp_flag' == 'read_cmp_target'.
 * If this is the case, then the decode function will be called.
 *************************************************************************************************/
    if (read_cmp_target == read_cmp_flag) {
        decodeMAX6675();
    }
}

MAX6675::~MAX6675()
{
    // TODO Auto-generated destructor stub
}

