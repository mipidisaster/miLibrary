/**************************************************************************************************
 * @file        GPIO.cpp
 * @author      Thomas
 * @version     V2.2
 * @date        22 Sept 2019
 * @brief       Source file for the Generic GPIO Class handle
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include <FileIndex.h>
#include FilInd_GPIO___HD

#if ( defined(zz__MiSTM32Fx__zz) || defined(zz__MiSTM32Lx__zz)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//==================================================================================================
GPIO::GPIO(GPIO_TypeDef *PortAddress, uint32_t pinnumber, Dir direction) {
/**************************************************************************************************
 * Create a GPIO class specific for the STM32F device
 * Receives the PortAddress pointer, and pin number - all comes from the cubeMX output
 * Also requires the direction of the pin - INPUT/OUTPUT
 * Not setup of the port clock or pin, covered by the cubeMX outputs
 *************************************************************************************************/
    this->pinnumber     = pinnumber;    // copy data into class
    this->PortAddress   = PortAddress;  //
    this->pindirection  = direction;    //
}
#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
GPIO::GPIO(_GPIOValue pinvalue, uint32_t pinnumber, _GPIODirec pindirection) {
/**************************************************************************************************
 * Create a GPIO class specific for the Raspberry Pi
 * Receives the initial pin state, pin number, and the direction - manually entered
 *************************************************************************************************/
    this->pinnumber     = pinnumber;    // copy data into class
    this->pindirection  = pindirection; //
    this->pinvalue      = pinvalue;     //

    if (this->pindirection == GPIO_IN)          // If GPIO Mode is set for INPUT - GPIO_IN
        pinMode((int)this->pinnumber, INPUT);   // Configure for INPUT
    else {                                      // If GPIO Mode is set for OUTPUT - GPIO_OUT
        pinMode((int)this->pinnumber, OUTPUT);  // Configure for OUTPUT
        this->setValue(this->pinvalue);         // Set the pin for initial state
    }
}
#else
//==================================================================================================
GPIO::GPIO(uint32_t pinnumber, _GPIODirec pindirection) {
    this->pinnumber     = pinnumber;    //
    this->pindirection  = direction;    //

}
#endif

uint8_t GPIO::toggleOutput() {
/**************************************************************************************************
 * Toggle the class linked pin, ONLY if the class has been declared as an OUTPUT.
 * Otherwise return error
 *************************************************************************************************/
    if (this->pindirection != Dir::OUTPUT)  // Check direction of pin, if not equal to OUTPUT
        return -1;                          // return error '-1'

#if ( defined(zz__MiSTM32Fx__zz) || defined(zz__MiSTM32Lx__zz)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//==================================================================================================
    this->PortAddress->ODR  ^= this->pinnumber;             // Toggle specific pin

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    if (this->pinvalue == GPIO_LOW)     // If the Pin is set at LOW
        this->setValue(GPIO_HIGH);      // Update to HIGH, function also updates the "pinvalue"
    else                                // If the Pin is set at HIGH
        this->setValue(GPIO_LOW);       // Update to LOW, function also updates the "pinvalue"

#else
//==================================================================================================

#endif

    return 0;
}

uint8_t GPIO::setValue(State value) {
/**************************************************************************************************
 * Set the state of the output pin as per user demand "value", ONLY if the class has been declared
 * as an OUTPUT
 * Otherwise return error
 *************************************************************************************************/
    if (this->pindirection != Dir::OUTPUT)  // Check direction of pin, if not equal to OUTPUT
        return -1;                          // return error '-1'

#if ( defined(zz__MiSTM32Fx__zz) || defined(zz__MiSTM32Lx__zz)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//==================================================================================================
    if (value == State::LOW)        // If demand is to set it LOW
        this->PortAddress->BSRR     =   (uint32_t)this->pinnumber << 16U;
            // Set the corresponding RESET bit within the BSRR register (shifted up by 16bits)
    else
        this->PortAddress->BSRR     =   this->pinnumber;
            // Set the corresponding SET bit within the BSRR register

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    if (value == GPIO_LOW)                          // If demand for Pin is set for LOW - GPIO_LOW
        digitalWrite((int)this->pinnumber, LOW);    // Drive the pin LOW
    else
        digitalWrite((int)this->pinnumber, HIGH);   // Drive the pin HIGH

    this->pinvalue      = value;                    // Update pin value

#else
//==================================================================================================

#endif

    return 0;
}

GPIO::State GPIO::getValue() {
/**************************************************************************************************
 * Read the state of the input pin. Function will only work on INPUT pin, if pin is declared as an
 * OUTPUT it will return an error
 *************************************************************************************************/
    if (this->pindirection != Dir::INPUT)   // Check direction of pin, if not equal to INPUT
        return State::LOW;                  // return LOW state

#if ( defined(zz__MiSTM32Fx__zz) || defined(zz__MiSTM32Lx__zz)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//==================================================================================================

    if (( this->PortAddress->IDR & this->pinnumber ) != 0 )
            // Check the state of the pin, and if it is set (i.e. not equal to zero), then output
            // HIGH status
        return State::HIGH;
    else    // Otherwise output LOW status
        return State::LOW;

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    return GPIO_LOW;
#else
//==================================================================================================
    return LOW;
#endif
}

GPIO::~GPIO()
{
    // TODO Auto-generated destructor stub
}
