/**************************************************************************************************
 * @file        DeMux.h
 * @author      Thomas
 * @version     V0.1
 * @date        2 Jun 2018
 * @brief       Header file for the Demultiplexor Class handle
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * This class utilises the peripheral class - GPIO to control an external Demultiplexor. A
 * Demultiplexor (DeMux) takes a single input and converts this into multiple. This class supports
 * the control of this through:
 *      High Enable pin     - Some DeMux have an enable pin that needs to be high for the output to
 *                            be changed
 *      Low Enable pin      - Same as above, however pin needs to be pulled low to change output
 *      Switches            - Selection entry for which output of the DeMux needs to be changed
 *
 * Use of class
 *      Initial call to contain pointers for the High/Low enable pins, Switches ARRAY, and the
 *      number of switches (size of ARRAY).
 **      **NOTE***
 **         If only one or neither of the enable pins are required, then provide __null
 **
 **     Once configured, the DeMux will be configured to be initially disabled. It can be enabled
 **     again via ".enable", and disabled via ".disable"
 **     The Selection can then be updated via ".updateselected", so long as the number provided is
 **     within the capability of the switches, then no faults will be set
 **
 **     There is no other functionality within this class
 *************************************************************************************************/
#ifndef DEMUX_H_
#define DEMUX_H_

#include <stdint.h>         // Defines types - uint8_t, etc.
#include "GPIO/GPIO.h"      // Defines class - GPIO

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
// None

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
// None

#else
//==================================================================================================
#error "Unrecognised target device"

#endif

// Defines specific within this class
//	--- Include any #defines within class - use similar naming as class
//	--- keep this relatively light, large quantities should be within a seperate header file
//	--- named <class_name>_defs.h

// Types used within this class
typedef enum {  // Enumerate type for showing status of the Demultiplexor
        DeMux_Enabled = 0,              // Indicate that Demultiplexor is Enabled
        DeMux_Disabled = !DeMux_Enabled // Indicate that Demultiplexor is Disabled
} _DeMuxState;

typedef enum {  // Enumerate type for showing status of the Demultiplexor
        DeMux_NoFault = 0,              // No fault with Demultiplexor
        DeMux_IncorrectSelection = 1,   // Fault with the setting of the Demultiplexor

        DeMux_Initialised = -1          // If initialised, this flag is set
} _DeMuxFlt;

class DeMux {
    private:
        GPIO            *Mux_LEnable;   // GPIO pin for Enabling Demux (Low enable)     [pointer]
        GPIO            *Mux_HEnable;   // GPIO pin for Enabling Demux (High enable)    [pointer]
        GPIO            *Mux_A;         // GPIO pin's for changing output of Demultiplexor
                                        // this is expected to be an array              [pointer]

        uint8_t         inputsize;      // Contain the number of input switches to Demux

    public:
        _DeMuxState     Status;         // Indicate the status of the Demultiplexor
        uint8_t         Selection;      // Indication of the current selected output
        _DeMuxFlt       Flt;            // Fault status of the Demultiplexor

        DeMux(GPIO *High_Enable, GPIO *Low_Enable, GPIO Switches[], uint8_t SwitchSize);
        void enable(void);              // Enable the Demultiplexor
        void disable(void);             // Disable the Demultiplexor
        _DeMuxFlt updateselection(uint8_t newselection);    // Update the selection


        virtual ~DeMux();
};

#endif /* DEMUX_H_ */
