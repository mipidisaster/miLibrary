/**************************************************************************************************
 * @file        GPIO.cpp
 * @author      Thomas
 * @brief       Source file for the Generic GPIO Class handle
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include <FileIndex.h>                  // Header for miLibrary index
#include FilInd_GPIO___HD               // Header for GPIO

// C System Header(s)
// ------------------
#include <stdint.h>

// C++ System Header(s)
// --------------------

// Other Libraries
// --------------
#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
#include "stm32f1xx_hal.h"              // Include the HAL library

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
#include "stm32l4xx_hal.h"              // Include the HAL library

#elif (zz__MiEmbedType__zz == 10)       // If the target device is an Raspberry Pi then
//=================================================================================================
#include <wiringPi.h>                   // Include the wiringPi library

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

//=================================================================================================

#if   ( (zz__MiEmbedType__zz == 50) || (zz__MiEmbedType__zz == 51)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================
GPIO::GPIO(GPIO_TypeDef *PortAddress, uint32_t pinnumber, Dir direction) {
/**************************************************************************************************
 * Create a GPIO class specific for the STM32F device
 * Receives the PortAddress pointer, and pin number - all comes from the cubeMX output
 * Also requires the direction of the pin - INPUT/OUTPUT
 * Not setup of the port clock or pin, covered by the cubeMX outputs
 *************************************************************************************************/
    _pin_number_      = pinnumber;      // copy data into class
    _port_address_    = PortAddress;    //
    _pin_direction_   = direction;      //
}
#else   // Raspberry Pi or Default build configuration
//=================================================================================================
GPIO::GPIO(State pinvalue, uint32_t pinnumber, Dir pindirection) {
/**************************************************************************************************
 * Create a GPIO class specific for the Raspberry Pi/Default build
 * Receives the initial pin state, pin number, and the direction - manually entered
 *************************************************************************************************/
    _pin_number_      = pinnumber;      // copy data into class
    _pin_direction_   = pindirection;   //
    _pin_value_       = pinvalue;       //

#if  (zz__MiEmbedType__zz == 10)        // If configured for RaspberryPi, then use wiringPi
    if (_pin_direction_ == Dir::kInput)             // If GPIO Mode is set for INPUT - kInput
        pinMode((int)_pin_number_, INPUT);          // Configure for INPUT
    else {                                          // If GPIO Mode is set for OUTPUT - kOutput
        pinMode((int)pinnumber, OUTPUT);            // Configure for OUTPUT
        setValue(_pin_value_);                      // Set the pin for initial state
    }
#endif
}
#endif

uint8_t GPIO::toggleOutput() {
/**************************************************************************************************
 * Toggle the class linked pin, ONLY if the class has been declared as an OUTPUT.
 * Otherwise return error
 *************************************************************************************************/
    if (_pin_direction_ != Dir::kOutput)    // Check direction of pin, if not equal to OUTPUT
        return -1;                          // return error '-1'

#if   ( (zz__MiEmbedType__zz == 50) || (zz__MiEmbedType__zz == 51)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================
    _port_address_->ODR  ^= _pin_number_;   // Toggle specific pin

#elif (zz__MiEmbedType__zz == 10)       // If the target device is an Raspberry Pi then
//=================================================================================================
    if (_pin_value_ == State::kLow) // If the Pin is set at LOW
        setValue(State::kHigh);     // Update to HIGH, function also updates the "pinvalue"
    else                            // If the Pin is set at HIGH
        setValue(State::kLow);      // Update to LOW, function also updates the "pinvalue"

#else
//=================================================================================================
// As is default configuration; there is no hardware to interact with - do nothing

#endif

    return 0;
}

uint8_t GPIO::setValue(State value) {
/**************************************************************************************************
 * Set the state of the output pin as per user demand "value", ONLY if the class has been declared
 * as an OUTPUT
 * Otherwise return error
 *************************************************************************************************/
    if (_pin_direction_ != Dir::kOutput)    // Check direction of pin, if not equal to OUTPUT
        return -1;                          // return error '-1'

#if   ( (zz__MiEmbedType__zz == 50) || (zz__MiEmbedType__zz == 51)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================
    if (value == State::kLow)           // If demand is to set it LOW
        _port_address_->BSRR     =   (uint32_t)_pin_number_ << 16U;
            // Set the corresponding RESET bit within the BSRR register (shifted up by 16bits)
    else
        _port_address_->BSRR     =   _pin_number_;
            // Set the corresponding SET bit within the BSRR register

#else       // Construction of class for 'Default' or RaspberryPi is the same
//=================================================================================================
#if  (zz__MiEmbedType__zz == 10)        // If configured for RaspberryPi, then use wiringPi
    if (value == State::kLow)                   // If demand for Pin is set for LOW - GPIO_LOW
        digitalWrite((int)_pin_number_, LOW);   // Drive the pin LOW
    else
        digitalWrite((int)_pin_number_, HIGH);  // Drive the pin HIGH
#endif

    _pin_value_     = value;                    // Update pin value

#endif

    return 0;
}

GPIO::State GPIO::getValue() {
/**************************************************************************************************
 * Read the state of the input pin. Function will only work on INPUT pin, if pin is declared as an
 * OUTPUT it will return an error
 *************************************************************************************************/
    if (_pin_direction_ != Dir::kInput)     // Check direction of pin, if not equal to INPUT
        return State::kLow;                 // return LOW state

#if   ( (zz__MiEmbedType__zz == 50) || (zz__MiEmbedType__zz == 51)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================

    if (( _port_address_->IDR & _pin_number_ ) != 0 )
            // Check the state of the pin, and if it is set (i.e. not equal to zero), then output
            // HIGH status
        return State::kHigh;
    else    // Otherwise output LOW status
        return State::kLow;

#else       // Construction of class for 'Default' or RaspberryPi is the same
//=================================================================================================
    return State::kLow;

#endif
}

GPIO::~GPIO()
{
    // TODO Auto-generated destructor stub
}
