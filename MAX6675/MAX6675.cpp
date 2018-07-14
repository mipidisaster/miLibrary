/**************************************************************************************************
 * @file        MAX6675.cpp
 * @author      Thomas
 * @version     V0.2
 * @date        13 Jul 2018
 * @brief       Header file for the MAX6675 Thermocouple Digital Converter
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include "MAX6675/MAX6675.h"

void MAX6675::InitSetup(SPIDevice *Device) {
/**************************************************************************************************
 * Default setup of MAX6675, where the hardware enabled chip select of SPI device is assumed
 * enabled, so only the desired SPI Device needs to be linked to this class.
 *      Will set the GPIOSelect pointer to null, and initial temperature to -999 and fault state
 *      to -1 (faulty)
 *************************************************************************************************/
    this->SPInterface   = Device;       // Copy across the SPI Device pointer to class
    this->GPIOSelect    = __null;       // As function call hasn't included GPIOSelect, link NULL
    this->DeMuxDevice   = __null;       // As function call hasn't included DeMux, link NULL

    this->DeMuxCS       = -1;           // Setup the Chip Select to default number

    this->Temp          = -999;         // Default Temperature to -999Degrees
    this->rawData       = 0;            // Initialise to zero
    this->Flt           = MAX6675_Initialised;  // Indicate initialised state for fault flag
}

MAX6675::MAX6675(SPIDevice *Device) {
/**************************************************************************************************
 * Call normal "InitSetup" function, and doesn't override anything else
 *************************************************************************************************/
    this->InitSetup(Device);            // Call default setup, to populate generic entries
}

MAX6675::MAX6675(SPIDevice *Device, GPIO *ChipSelect) {
/**************************************************************************************************
 * Overloaded function with both SPIDevice and GPIO Class pointers. With this call the class will
 * assume that the provided GPIO pointer is the Chip Select pin for the MAX6675 device.
 * Everything else is the same between this call and the other version
 *************************************************************************************************/
    this->InitSetup(Device);            // Call default setup, to populate generic entries

    this->GPIOSelect    = ChipSelect;   // Copy across the GPIO Chip Select pointer
}

MAX6675::MAX6675(SPIDevice *Device, DeMux *DeMuxCS, uint8_t CSNum) {
/**************************************************************************************************
 * Overloaded function with both SPIDevice and DeMux Class pointers.
 * With this call input DeMux entries will be stored within the class, and used as the only form
 * of communication with the selected MAX6675 device
 *************************************************************************************************/
        this->InitSetup(Device);            // Call default setup, to populate generic entries

        this->DeMuxDevice   = DeMuxCS;      // Copy across the DeMux pointer
        this->DeMuxCS       = CSNum;        // Store the DeMux selection number for MAX6675
}

_MAX6675Flt MAX6675::readTemp(void) {
/**************************************************************************************************
 * Function will transfer dummy data to MAX6675 to read the contents of its register. Then read
 * the contents of this and calculate the temperature in celsius.
 * Any faults with the transfer or incorrect register state will be invidated via the fault flag
 * "TempFlt"
 *
 * Depending upon how the MAX6675 was initialised, it will call different variations of the
 * SPIDevice - SPITransfer, either normally (hardware chip select), or with the linked GPIO
 * (software chip select)
 *************************************************************************************************/
    uint8_t     data[2] = { 0 };        // Array for data from MAX6675
    uint8_t     returnvalue = 0;        // Value returned from function
    uint16_t    registerdata = 0;       // Data to store the MAX6675 register

    // Determine if need to use Chip Select
    if      (this->GPIOSelect != __null) {      // If GPIO has been provided
        returnvalue = this->SPInterface->SPITransfer(this->GPIOSelect, data, 2);
                // Provide GPIO class to overloaded function call
    }

    else if (this->DeMuxDevice != __null) {     // If DeMux has been provided
        returnvalue = this->SPInterface->SPITransfer(this->DeMuxDevice, this->DeMuxCS,
                                                     data, 2);
                // Provide both the DeMux class and selection entry to overloaded function call
    }

    else {    // If no Chip Select functionality has been provided then, it is not required
        returnvalue = this->SPInterface->SPITransfer(data, 2);
    }

    registerdata = ((uint16_t)data[0] << 8) | data[1];      // Shift data into 16bit data
    this->rawData = registerdata;                           // Copy data into class

    if (returnvalue != 0) {                 // If the write fails then
        this->Flt = MAX6675_ReadError;      // Indicate fault
        return (this->Flt);                 // Return fault state
    }

    if ((registerdata & (MAX6675_Dummy | MAX6675_NoThermo | MAX6675_DevID)) != 0x0000) {
        // If the data read contains an incorrect value, then determine fault, and setup fault flag
        if ((registerdata & MAX6675_NoThermo) != 0x0000) {  // If open circuit on sensor
            this->Flt   = MAX6675_Nosensor;                 // Set no sensor fault
            return (this->Flt);                             // Return fault state
        }
        else if ((registerdata & MAX6675_Dummy) != 0x0000) {// If Dummy bit is incorrect
            this->Flt   = MAX6675_DummyFlt;                 // Set Dummy fault
            return (this->Flt);                             // Return fault state
        }
        else {                                              // If Device ID is incorrect
            this->Flt   = MAX6675_DeviceID;                 // Set Device ID fault
            return (this->Flt);                             // Return fault state
        }
    }
    // If have reached this point, then there is no recognised fault with the read/data
    registerdata &= MAX6675_Temp;       // Retain only the 12bits for temperature
    registerdata >>= 3;                 // Shift down by 3 bits
    this->Temp      = ((float)registerdata) * 0.25;
    // Resolution of data is 1 bit = 0.25Degrees, therefore multiple by 0.25 and convert to
    // float type (single precision)
    this->Flt   = MAX6675_NoFault;      // Indicate healthy data
    return (this->Flt);                 // Return good fault state
}

MAX6675::~MAX6675()
{
    // TODO Auto-generated destructor stub
}

