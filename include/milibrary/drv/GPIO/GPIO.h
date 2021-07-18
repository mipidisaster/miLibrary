/**************************************************************************************************
 * @file        GPIO.h
 * @author      Thomas
 * @brief       Header file for the Generic GPIO Class handle
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * Class supports multiple target devices, depending upon which one is defined will modify how the
 * GPIO class can be initialised.
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
//=================================================================================================
#include "stm32f1xx_hal.h"              // Include the HAL library

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
#include "stm32l4xx_hal.h"              // Include the HAL library

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================
#include <wiringPi.h>                   // Include the wiringPi library

#else
//=================================================================================================
#error "Unrecognised target device"

#endif

// Types used within this class
// Defined within the class, to ensure are contained within the correct scope

class GPIO {
/**************************************************************************************************
 * ==   TYPES   == >>>       TYPES GENERATED WITHIN CLASS        <<<
 *   -----------
 *  Following types are generated within this class. If needed outside of the class, need to
 *  state "GPIO::" followed by the type.
 *************************************************************************************************/
public:
        enum State : uint8_t  {   kLow = 0,   kHigh = 1 };
        enum Dir   : uint8_t  {kOutput = 0,  kInput = 1 };

/**************************************************************************************************
 * == GEN PARAM == >>>       GENERIC PARAMETERS FOR CLASS        <<<
 *   -----------
 *  Parameters required for the class to function.
 *************************************************************************************************/
    private:
        uint32_t        _pin_number_;
        Dir             _pin_direction_;

/**************************************************************************************************
 * == SPC PARAM == >>>        SPECIFIC ENTRIES FOR CLASS         <<<
 *   -----------
 *  Following are functions and parameters which are specific for the embedded device selected.
 *  The initialisation function for the class is also within this section, which again will be
 *  different depending upon the embedded device selected.
 *************************************************************************************************/

#if ( defined(zz__MiSTM32Fx__zz) || defined(zz__MiSTM32Lx__zz)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================
    private:
        GPIO_TypeDef    *_port_address_;        // Store the Port Address of pin

    public:
        GPIO(GPIO_TypeDef *PortAddress, uint32_t pinnumber, Dir direction);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================
    private:
        _GPIOValue      _pin_value_;            // Current value of pin

    public:
    GPIO(_GPIOValue pinvalue, uint32_t pinnumber, _GPIODirec pindirection);

#else
//=================================================================================================
    public:
        GPIO(uint32_t pinnumber, _GPIODirec pindirection);

#endif

/**************************************************************************************************
 * == GEN FUNCT == >>>      GENERIC FUNCTIONS WITHIN CLASS       <<<
 *   -----------
 *  The following are functions scoped within the "SPIPeriph" class, which are generic; this means
 *  are used by ANY of the embedded devices supported by this class.
 *  The internals of the class, will then determine how it will be managed between the multiple
 *  embedded devices.
 *************************************************************************************************/

public:     /**************************************************************************************
             * == PROTECTED == >>>     DIRECT HARDWARE READING FUNCTIONS     <<<
             *   -----------
             * As the GPIO is relatively low level, there is no need to separate out functions
             * which are/are not visible outside of this class
             *************************************************************************************/
        // Output controls
        virtual uint8_t toggleOutput();                 // Toggle the target GPIO
        virtual uint8_t setValue(State value);


        // Input controls
        virtual State getValue();
        virtual ~GPIO();
};

#endif /* GPIO_H_ */
