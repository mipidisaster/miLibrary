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
// None required

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

void GPIO::piSetup(void) {
/**************************************************************************************************
 * Static function which wraps the 'wiringPiSetup' routines within my 'GPIO' class.
 * Function is to only be called once, otherwise it will run out of file handles, so be careful
 * when using
 *************************************************************************************************/
#if  (zz__MiEmbedType__zz == 10)        // If configured for RaspberryPi, then use wiringPi
    wiringPiSetupGpio();
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

void GPIO::setGPIOArray(GPIO *gpioarray, uint32_t number_gpios, uint32_t value) {
/**************************************************************************************************
 * STATIC function used to set multiple gpio's in an array at the same time
 * First entry within array 'gpioarray' is the LSB
 *************************************************************************************************/
    uint32_t temp = 0;      // Temporary variable used within this function, for
                            //  -> Determining maximum size of gpioarray selection
                            //  -> Looping through input selection, to set corresponding switch

    temp = ((1 << number_gpios) - 1);       // Calculate the maximum size, by shifting 0x01 up by
                                            // number of switches provided. Then subtracting 1
                                            //      equivalent to (2^x) - 1.

    // Force the input to be within the limits of number of GPIO pins provided
    value &= temp;

    // If get to this point, then input selection is within capabilitys of the class setup
    for (temp = 0; temp != (number_gpios); temp++) {    // Now use temporary variable to loop
        if (temp != 0) {                        // If not the first pass of loop then
            value >>= 1;                        // binary shift the input selection number by 1
        }                                       // to the right (make it smaller)

        if (value & 1)                              // If lowest bit is "1", then
            gpioarray[temp].setValue(GPIO::kHigh);  // Set corresponding switch "HIGH"
        else                                        // If lowest bit is "0", then
            gpioarray[temp].setValue(GPIO::kLow);   // Set corresponding switch "LOW"
    }
}

GPIO::~GPIO() {
/**************************************************************************************************
 * When the destructor is called, need to ensure that the memory allocation is cleaned up, so as
 * to avoid "memory leakage"
 *************************************************************************************************/
#if   ( (zz__MiEmbedType__zz == 50) || (zz__MiEmbedType__zz == 51)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================
// Nothing needs to be done

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
// Nothing needs to be done

#endif
    // TODO Auto-generated destructor stub
}
