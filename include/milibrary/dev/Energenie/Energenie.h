/**************************************************************************************************
 * @file        Energenie.h
 * @author      Thomas
 * @brief       << Manually Entered >>
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * 
 * 
 *************************************************************************************************/
#ifndef ENERGENIE_H_
#define ENERGENIE_H_

#include "GPIO/GPIO.h"
#include <wiringPi.h>

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
#error "Not supported"

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================
// Add includes specific to the Raspberry Pi

#else
//=================================================================================================
#error "Unrecognised target device"

#endif

// Defines specific within this class
//	--- Include any #defines within class - use similar naming as class
//	--- keep this relatively light, large quantities should be within a seperate header file
//	--- named <class_name>_defs.h

// Types used within this class
//	--- Include an types used within this class, put "None" if not required

class Energenie {
private:
    GPIO    *ModulationEnab;        // Pointer to the GPIO to enable/disable modulation
    GPIO    *ASKSelector;           // Pointer to the GPIO to select ASK/FSK

    GPIO    *D0;                    // Pointer to D0
    GPIO    *D1;                    // Pointer to D1
    GPIO    *D2;                    // Pointer to D2
    GPIO    *D3;                    // Pointer to D3

public:
    Energenie();
    void SwitchOn();
    void SwitchOff();
    virtual ~Energenie();
};

#endif /* ENERGENIE_H_ */
