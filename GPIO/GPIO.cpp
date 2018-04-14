/**************************************************************************************************
 * @fileh       GPIO.cpp
 * @author      Thomas
 * @version     V0.1
 * @date        10 Apr 2018
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
 * Not setup of the port clock or pin, covered by the cubeMX outputs
 *************************************************************************************************/
    this->pinnumber     = pinnumber;    // copy data into class
    this->PortAddress   = PortAddress;  //
    this->pindirection  = direction;    //
}
#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
GPIO::GPIO(int pinnumber)
{
    this->pinnumber = pinnumber;

}
#else
//==================================================================================================
GPIO::GPIO(int pinnumber)
{
    this->pinnumber = pinnumber;

}
#endif

uint8_t GPIO::toggleOutput() {
/**************************************************************************************************
 * Toggle the class linked pin, ONLY if the class has been declared as an OUTPUT.
 * Otherwise return error
 *************************************************************************************************/
    if (this->pindirection != OUTPUT)   // Check direction of pin, if not equal to OUTPUT
        return -1;                      // return error '-1'

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    HAL_GPIO_TogglePin(this->PortAddress, this->pinnumber); // Use HAL function to toggle pin

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================

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
    if (this->pindirection != OUTPUT)   // Check direction of pin, if not equal to OUTPUT
        return -1;                      // return error '-1'

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    if (value == LOW)           // If demand is to set it LOW
        HAL_GPIO_WritePin(this->PortAddress, this->pinnumber, GPIO_PIN_RESET);
        // use HAL function to drive pin low
    else
        HAL_GPIO_WritePin(this->PortAddress, this->pinnumber, GPIO_PIN_SET);
        // use HAL function to drive pin High

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================

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
    if (this->pindirection != INPUT)    // Check direction of pin, if not equal to INPUT
        return LOW;                     // return LOW state

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    if (HAL_GPIO_ReadPin(this->PortAddress, this->pinnumber) == GPIO_PIN_RESET)
            // Check the state of the pin, and if it is RESET, then output LOW status
        return LOW;
    else    // Otherwise output HIGH status
        return HIGH;

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================

#else
//==================================================================================================

#endif
}

GPIO::~GPIO()
{
    // TODO Auto-generated destructor stub
}
