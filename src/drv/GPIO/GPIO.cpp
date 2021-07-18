/**************************************************************************************************
 * @file        GPIO.cpp
 * @author      Thomas
 * @brief       Source file for the Generic GPIO Class handle
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include <FileIndex.h>
#include FilInd_GPIO___HD

#if ( defined(zz__MiSTM32Fx__zz) || defined(zz__MiSTM32Lx__zz)  )
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
#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================
GPIO::GPIO(_GPIOValue pinvalue, uint32_t pinnumber, _GPIODirec pindirection) {
/**************************************************************************************************
 * Create a GPIO class specific for the Raspberry Pi
 * Receives the initial pin state, pin number, and the direction - manually entered
 *************************************************************************************************/
    _pin_number_      = pinnumber;      // copy data into class
    _pin_direction_   = pindirection;   //
    _pin_value_       = pinvalue;       //

    if (_pin_direction_ == GPIO_IN)         // If GPIO Mode is set for INPUT - GPIO_IN
        pinMode((int)_pin_number_, INPUT);  // Configure for INPUT
    else {                                  // If GPIO Mode is set for OUTPUT - GPIO_OUT
        pinMode((int)pinnumber, OUTPUT);    // Configure for OUTPUT
        setValue(_pin_number_);             // Set the pin for initial state
    }
}
#else
//=================================================================================================
GPIO::GPIO(uint32_t pinnumber, _GPIODirec pindirection) {
    pinnumber     = pinnumber;      //
    pindirection  = direction;      //

}
#endif

uint8_t GPIO::toggleOutput() {
/**************************************************************************************************
 * Toggle the class linked pin, ONLY if the class has been declared as an OUTPUT.
 * Otherwise return error
 *************************************************************************************************/
    if (_pin_direction_ != Dir::kOutput)    // Check direction of pin, if not equal to OUTPUT
        return -1;                          // return error '-1'

#if ( defined(zz__MiSTM32Fx__zz) || defined(zz__MiSTM32Lx__zz)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================
    _port_address_->ODR  ^= _pin_number_;   // Toggle specific pin

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================
    if (_pin_value_ == GPIO_LOW)    // If the Pin is set at LOW
        setValue(GPIO_HIGH);        // Update to HIGH, function also updates the "pinvalue"
    else                            // If the Pin is set at HIGH
        setValue(GPIO_LOW);         // Update to LOW, function also updates the "pinvalue"

#else
//=================================================================================================

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

#if ( defined(zz__MiSTM32Fx__zz) || defined(zz__MiSTM32Lx__zz)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================
    if (value == State::kLow)           // If demand is to set it LOW
        _port_address_->BSRR     =   (uint32_t)_pin_number_ << 16U;
            // Set the corresponding RESET bit within the BSRR register (shifted up by 16bits)
    else
        _port_address_->BSRR     =   _pin_number_;
            // Set the corresponding SET bit within the BSRR register

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================
    if (value == GPIO_LOW)                      // If demand for Pin is set for LOW - GPIO_LOW
        digitalWrite((int)_pin_number_, LOW);   // Drive the pin LOW
    else
        digitalWrite((int)_pin_number_, HIGH);  // Drive the pin HIGH

    pinvalue      = value;                    // Update pin value

#else
//=================================================================================================

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

#if ( defined(zz__MiSTM32Fx__zz) || defined(zz__MiSTM32Lx__zz)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================

    if (( _port_address_->IDR & _pin_number_ ) != 0 )
            // Check the state of the pin, and if it is set (i.e. not equal to zero), then output
            // HIGH status
        return State::kHigh;
    else    // Otherwise output LOW status
        return State::kLow;

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================
    return GPIO_LOW;
#else
//=================================================================================================
    return LOW;
#endif
}

GPIO::~GPIO()
{
    // TODO Auto-generated destructor stub
}
