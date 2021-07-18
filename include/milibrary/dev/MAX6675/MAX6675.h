/**************************************************************************************************
 * @file        MAX6675.h
 * @author      Thomas
 * @brief       Source file for the MAX6675 Thermocouple Digital Converter
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * This class utilises the peripheral classes - SPIDevice to determine which SPI peripheral to
 * use. The class caller is overloaded, so that if the GPIO class is also used it will then use
 * this as the Chip Select for the MAX6675 device.
 * Use of class
 *      Initial call to contain a pointer to the desired SPIDevice
 *          Optional -> If hardware chip select is not used, providing a pointer to the desired
 *                      GPIO class, will use this to interface to MAX6675
 *
 *      Update the temperature and fault reading through "readTemp", this will then update
 *      "Temp" and "TempFlt".
 *          TempFlt = -1 for faulty data, or 0 for healthy data
 *
 *      There is no other functionality within this class
 *************************************************************************************************/
#ifndef MAX6675_MAX6675_H_
#define MAX6675_MAX6675_H_

#include <stdint.h>
#include "SPIDevice/SPIDevice.h"        // Allow use of SPI class
#include "GPIO/GPIO.h"                  // Allow use of GPIO class, for Chip Select
#include "DeMux/DeMux.h"                // Allow use of the DeMux class, for Chip Select


#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
// None

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================
// None

#else
//=================================================================================================
#error "Unrecognised target device"

#endif

// Defines specific within this class
#define MAX6675_Dummy       0x8000      // Bit 15 set TRUE - Dummy bit incorrect
#define MAX6675_NoThermo    0x0004      // Bit 2  set TRUE - Indicates open circuit on thermocouple
#define MAX6675_DevID       0x0002      // Bit 1  set TRUE - Device ID incorrect

#define MAX6675_Temp        0x7FF8      // 12bit position for the temperature

// Types used within this class
typedef enum {  // Enumerate type for showing status of the MAX6675 device
    MAX6675_NoFault = 0,            // No fault present with device
    MAX6675_ReadError = 1,          // Error with the transmission of data to device
    MAX6675_DummyFlt  = 2,          // Dummy bit is set incorrectly
    MAX6675_DeviceID  = 3,          // Device ID is incorrect
    MAX6675_Nosensor  = 4,          // Open circuit on the thermocouple (no thermocouple attached)

    MAX6675_Initialised = -1        // If initialised, this flag is set
} _MAX6675Flt;

class MAX6675 {
    private:
        SPIDevice       *SPInterface;   // Linked SPI Interface class
        GPIO            *GPIOSelect;    // Optional GPIO class for Chip Select

        DeMux           *DeMuxDevice;   // Optional DeMux class for Chip Select
        uint8_t         DeMuxCS;        // Optional DeMux selection number

        void            InitSetup(SPIDevice *Device);   // Setup basic parameters (SPIDevice,
                                                        // temperature, etc.

    public:
        float           Temp;           // Calculated temperature (celsius) from last "readTemp"
        _MAX6675Flt     Flt;            // Fault indication of Temperature from last "readTemp"
        uint16_t        rawData;        // Raw data read from MAX6675

        MAX6675(SPIDevice *Device);
        MAX6675(SPIDevice *Device, GPIO *ChipSelect);
        MAX6675(SPIDevice *Device, DeMux *DeMuxCS, uint8_t CSNum);

        _MAX6675Flt readTemp(void);


        virtual ~MAX6675();
};

#endif /* MAX6675_MAX6675_H_ */
