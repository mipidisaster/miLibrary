/**************************************************************************************************
 * @file        GPIO.h
 * @author      Thomas
 * @version     V0.4
 * @date        16 Apr 2018
 * @brief       Header file for the Generic GPIO Class handle
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * Class multiple target devices, depending upon which one is defined will modify how the GPIO
 * class can be initialised.
 * The basic use of the class is the same for all target devices
 *      Call Class GPIO to initialise the class, and link the pinnumber and direction
 *          STM devices will require the PORT address as well
 *
 *      To change the state of the pin either use .toggleOutput or .setValue(HIGH/LOW) for outputs
 *      To read the state of the pin use getValue() for inputs
 *
 *      There is no other functionality within this class
 *************************************************************************************************/

#ifndef GPIO_H_
#define GPIO_H_

#include <stdint.h>
#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
#include "stm32f1xx_hal.h"              // Include the HAL library

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
#include <wiringPi.h>                   // Include the wiringPi library

#else
//==================================================================================================
#error "Unrecognised target device"

#endif

// Types used within this class
typedef enum GPIO_VALUE { GPIO_LOW = 0, GPIO_HIGH = ~GPIO_LOW } _GPIOValue;
typedef enum GPIO_DIREC { GPIO_OUT = 0, GPIO_IN = ~GPIO_OUT} _GPIODirec;

class GPIO {
    // Declarations which are generic, and will be used in ALL devices
    private:
        uint32_t        pinnumber;
        _GPIODirec      pindirection;

// Device specific entries
#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    private:
        GPIO_TypeDef    *PortAddress;           // Store the Port Address of pin

    public:
    GPIO(GPIO_TypeDef *PortAddress, uint32_t pinnumber, _GPIODirec direction);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    private:
        _GPIOValue      pinvalue;               // Current value of pin

    public:
    GPIO(_GPIOValue pinvalue, uint32_t pinnumber, _GPIODirec pindirection);

#else
//==================================================================================================
    public:
        GPIO(uint32_t pinnumber, _GPIODirec pindirection);

#endif

    public:
    // Functions which are generic for any device being controlled
        // Output controls
        virtual uint8_t toggleOutput();
        virtual uint8_t setValue(_GPIOValue value);


        // Input controls
        virtual _GPIOValue getValue();
        virtual ~GPIO();
};

#endif /* GPIO_H_ */
