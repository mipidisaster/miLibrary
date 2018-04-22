/**************************************************************************************************
 * @file        MAX6675.cpp
 * @author      Thomas
 * @version     V0.1
 * @date        22 Apr 2018
 * @brief       Header file for the MAX6675 Thermocouple Digital Converter
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include "MAX6675/MAX6675.h"

MAX6675::MAX6675(SPIDevice *Device) {
/**************************************************************************************************
 * Default setup of MAX6675, where the hardware enabled chip select of SPI device is assumed
 * enabled, so only the desired SPI Device needs to be linked to this class.
 *      Will set the GPIOSelect pointer to null, and initial temperature to -999 and fault state
 *      to -1 (faulty)
 *************************************************************************************************/
    this->SPInterface   = Device;       // Copy across the SPI Device pointer to class
    this->GPIOSelect    = __null;       // As function call hasn't included GPIOSelect, link NULL

    this->Temp          = -999;         // Default Temperature to -999Degrees
    this->TempFlt       = -1;           // Indicate faulty data (= -1)
}

MAX6675::MAX6675(SPIDevice *Device, GPIO *ChipSelect) {
/**************************************************************************************************
 * Overloaded function with both SPIDevice and GPIO Class pointers. With this call the class will
 * assume that the provided GPIO pointer is the Chip Select pin for the MAX6675 device.
 * Everything else is the same between this call and the other version
 *************************************************************************************************/
    this->SPInterface   = Device;       // Copy across the SPI Device pointer to class
    this->GPIOSelect    = ChipSelect;   // Copy across the GPIO Chip Select pointer

    this->Temp          = -999;         // Default Temperature to -999Degrees
    this->TempFlt       = -1;           // Indicate faulty data (= -1)
}

void MAX6675::readTemp(void) {
/**************************************************************************************************
 * Function will transfer dummy data to MAX6675 to read the contents of its register. Then read
 * the contents of this and calculate the temperature in celsius.
 * Any faults with the transfer or incorrect register data will invalidate the Temperature reading
 * (TempFlt = -1), a good read will indicate this as healthy (TempFlt = 0)
 *
 * Depending upon how the MAX6675 was initialised, it will call different variations of the
 * SPIDevice - SPITransfer, either normally (hardware chip select), or with the linked GPIO
 * (software chip select)
 *************************************************************************************************/
    uint8_t     data[2] = { 0 };        // Array for data from MAX6675
    uint8_t     returnvalue = 0;        // Value returned from function
    uint16_t    registerdata = 0;       // Data to store the MAX6675 register

    // Determine if need to use Chip Select
    if (this->GPIOSelect != __null) {           // If GPIO has been provided
        returnvalue = this->SPInterface->SPITransfer(this->GPIOSelect, data, 2);
                // Provide GPIO class to overloaded function call
    } else {    // If no Chip Select functionality has been provided then, it is not required
        returnvalue = this->SPInterface->SPITransfer(data, 2);
    }

    registerdata = ((uint16_t)data[0] << 8) | data[1];      // Shift data into 16bit data
    if ((returnvalue != 0) && ((registerdata & MAX6675_Faulty) != 0x0000)) {
        // If the transfer to MAX6675 failed, or the contents of the read register is wrong
        // then set fault state

        this->TempFlt = -1;         // Indicate temperature fault (= -1), and leave temperature
                                    // at last good state (by not changing it)
    } else {
        registerdata &= MAX6675_Temp;       // Retain only the 12bits for temperature
        registerdata >>= 3;                 // Shift down by 3 bits
        this->Temp      = ((float)registerdata) * 0.25;
        // Resolution of data is 1 bit = 0.25Degrees, therefore multiple by 0.25 and convert to
        // float type (single precision)
        this->TempFlt   = 0;                // Indicate healthy data (= 0)
    }
}

MAX6675::~MAX6675()
{
    // TODO Auto-generated destructor stub
}

