/**************************************************************************************************
 * @file        SPIDevice.h
 * @author      Thomas
 * @version     V0.4
 * @date        21 Apr 2018
 * @brief       Header file for the Generic SPIDevice Class handle
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * Class supports multiple target devices, depending upon which one is defined will modify how the
 * SPIDevice class can be initialised.
 * The basic used of the class is the same for all target devices
 *      Call Class SPIDevice to initialise the class
 *          For STM32F devices, provide the address of the SPI handler - from cubeMX
 *          For RaspberryPi, provide the channel, speed and mode
 *
 *      To transfer data to the selected slave device, a pointer to the array of data is required
 *      along with the intended number of bytes to be transmitted - function call "SPITransfer"
 *      This function is overloaded, so depending upon which addition arguments are provided will
 *      do more:
 *          If the GPIO Class is added at the start, it will be used as a software chip select,
 *          being pulled low prior to transfer of data, and then high after transfer is complete
 *
 *      There is no other functionality within this class
 *************************************************************************************************/
#ifndef SPIDEVICE_H_
#define SPIDEVICE_H_

#include <stdint.h>
#include "GPIO/GPIO.h"                  // Allow use of GPIO class, for Chip Select

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
#include "stm32f1xx_hal.h"              // Include the HAL library

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
#include <wiringPiSPI.h>                // Include the wiringPi SPI library

#else
//==================================================================================================
#error "Unrecognised target device"

#endif

// Types used within this class
typedef enum SPI_MODE { MODE0 = 0,      // Clock Idles at 0, and capture on the rising edge
                        MODE1 = 1,      // Clock Idles at 0, and capture on the falling edge
                        MODE2 = 2,      // Clock Idles at 1, and captures on the falling edge
                        MODE3 = 3       // Clock Idles at 1, and captures on the rising edge
                      } _SPIMode;

class SPIDevice {
    // Declarations which are generic, and will be used in ALL devices
    private:
        _SPIMode         Mode;

// Device specific entries
#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    private:
        SPI_HandleTypeDef   *SPIHandle;         // Store the Handle for the SPI Device, from cubeMX

    public:
        SPIDevice(SPI_HandleTypeDef *SPIHandle);


#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    private:
        int SPIChannel;                         // Store the channel used for SPI

    public:
        SPIDevice(int channel, int speed, _SPIMode Mode);

#else
//==================================================================================================
    public
        SPIDevice();

#endif
    private:
        uint8_t SPIRWTransfer(uint8_t *pData, uint16_t size);   // Private function to transmit
                // data via SPI

    public:
    // Functions which are generic for any device being controlled
/**************************************************************************************************
 * SPITransfer is an overloaded function, so therefore has multiple uses
 *  The first   is the "default", where only the data and size is provided
 *  The second  is provided the GPIO for Chip Select, which will be pulled low prior to transfer
 *              and then pushed back high after
 *************************************************************************************************/
        uint8_t SPITransfer(uint8_t *pData, uint16_t size);
        uint8_t SPITransfer(GPIO *ChipSelect, uint8_t *pData, uint16_t size);


        virtual ~SPIDevice();
};

#endif /* SPIDEVICE_H_ */
