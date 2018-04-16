/**************************************************************************************************
 * @file        GPIO.cpp
 * @author      Thomas
 * @version     V0.3
 * @date        14 Apr 2018
 * @brief       Source file for the Generic GPIO Class handle
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include "GPIO.h"

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
GPIO::GPIO(GPIO_TypeDef *PortAddress, uint32_t pinnumber, _GPIODirec direction) {
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

    if (this->pindirection == GPIO_IN)      // If GPIO Mode is set for INPUT - GPIO_IN
        pinMode(this->pinnumber, INPUT);    // Configure for INPUT
    else {                                  // If GPIO Mode is set for OUTPUT - GPIO_OUT
        pinMode(this->pinnumber, OUTPUT);   // Configure for OUTPUT
        this->setValue(this->pinvalue);     // Set the pin for initial state
    }
}
#else
//==================================================================================================
GPIO::GPIO(uint32_t pinnumber, _GPIODirec pindirection)
{
    this->pinnumber     = pinnumber;

}
#endif

uint8_t GPIO::toggleOutput() {
/**************************************************************************************************
 * Toggle the class linked pin, ONLY if the class has been declared as an OUTPUT.
 * Otherwise return error
 *************************************************************************************************/
    if (this->pindirection != GPIO_OUT) // Check direction of pin, if not equal to OUTPUT
        return -1;                      // return error '-1'

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    HAL_GPIO_TogglePin(this->PortAddress, this->pinnumber); // Use HAL function to toggle pin

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

uint8_t GPIO::setValue(_GPIOValue value) {
/**************************************************************************************************
 * Set the state of the output pin as per user demand "value", ONLY if the class has been declared
 * as an OUTPUT
 * Otherwise return error
 *************************************************************************************************/
    if (this->pindirection != GPIO_OUT) // Check direction of pin, if not equal to OUTPUT
        return -1;                      // return error '-1'

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    if (value == GPIO_LOW)      // If demand is to set it LOW
        HAL_GPIO_WritePin(this->PortAddress, this->pinnumber, GPIO_PIN_RESET);
        // use HAL function to drive pin low
    else
        HAL_GPIO_WritePin(this->PortAddress, this->pinnumber, GPIO_PIN_SET);
        // use HAL function to drive pin High

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    if (value == GPIO_LOW)                      // If demand for Pin is set for LOW - GPIO_LOW
        digitalWrite(this->pinnumber, LOW);     // Drive the pin LOW
    else
        digitalWrite(this->pinnumber, HIGH);    // Drive the pin HIGH

    this->pinvalue      = value;                // Update pin value

#else
//==================================================================================================

#endif

    return 0;
}

_GPIOValue GPIO::getValue() {
/**************************************************************************************************
 * Read the state of the input pin. Function will only work on INPUT pin, if pin is declared as an
 * OUTPUT it will return an error
 *************************************************************************************************/
    if (this->pindirection != GPIO_IN)  // Check direction of pin, if not equal to INPUT
        return GPIO_LOW;                // return LOW state

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    if (HAL_GPIO_ReadPin(this->PortAddress, this->pinnumber) == GPIO_PIN_RESET)
            // Check the state of the pin, and if it is RESET, then output LOW status
        return GPIO_LOW;
    else    // Otherwise output HIGH status
        return GPIO_HIGH;

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
