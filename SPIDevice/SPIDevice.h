/**************************************************************************************************
 * @file        SPIDevice.h
 * @author      Thomas
 * @version     V0.2
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
 *      To transfer data to the selected slave device (selection to be done outside of class)
 *      provide pointer to array, and indicate number of bytes to be transfered
 *
 *      There is no other functionality within this class
 *************************************************************************************************/
#ifndef SPIDEVICE_H_
#define SPIDEVICE_H_

#include <stdint.h>
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

    public:
    // Functions which are generic for any device being controlled
        uint8_t SPITransfer(uint8_t *pData, uint16_t size);
        virtual ~SPIDevice();
};

#endif /* SPIDEVICE_H_ */
