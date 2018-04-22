/**************************************************************************************************
 * @file        MAX6675.h
 * @author      Thomas
 * @version     V0.1
 * @date        22 Apr 2018
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

#define MAX6675_Faulty      0x8006      // Register faulty state
        // indicated by ->
        //                  Bit 15  set TRUE  - as dummy signal should be zero
        //                  Bit 2   set TRUE  - as this indicates an open circuit on thermocouple
        //                  Bit 1   set TRUE  - Device ID should be zero

#define MAX6675_Temp        0x7FF8      // 12bit position for the temperature

class MAX6675 {
    private:
        SPIDevice       *SPInterface;   // Linked SPI Interface class
        GPIO            *GPIOSelect;    // Optional GPIO class for Chip Select

    public:
        float           Temp;           // Calculated temperature (celsius) from last "readTemp"
        int8_t          TempFlt;        // Fault indication of Temperature from last "readTemp"

        MAX6675(SPIDevice *Device);
        MAX6675(SPIDevice *Device, GPIO *ChipSelect);

        void readTemp(void);


        virtual ~MAX6675();
};

#endif /* MAX6675_MAX6675_H_ */
