/**************************************************************************************************
 * @file        DeMux.h
 * @author      Thomas
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

#include "FileIndex.h"
#include <stdint.h>             // Defines types - uint8_t, etc.
#include FilInd_GPIO___HD       // Defines class - GPIO

#if ( defined(zz__MiSTM32Fx__zz) || defined(zz__MiSTM32Lx__zz)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================
// None

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================
// None

#else
//=================================================================================================
#error "Unrecognised target device"

#endif

// Defines specific within this class
// None

// Types used within this class
// Defined within the class, to ensure are contained within the correct scope

class DeMux {
/**************************************************************************************************
 * ==   TYPES   == >>>       TYPES GENERATED WITHIN CLASS        <<<
 *   -----------
 *  Following types are generated within this class. If needed outside of the class, need to
 *  state "DeMux::" followed by the type.
 *************************************************************************************************/
public:
    enum class DevFlt : uint8_t {       // Fault Type of the class (internal enumerate)
        kNone                   = 0x00, // Normal Operation
        kIncorrect_Selection    = 0x01, // Fault with the setting of the Demultiplexor

        kInitialised            = 0xFF  // Just initialised
    };

    enum class DevState : uint8_t {     // Enumerate type for showing state of the Demultiplexor
        kEnable     = 0x00,             //  Indicate that Demultiplexor is Enabled
        kDisable    = 0xFF              //  Indicate that Demultiplexor is Disabled
    };

    private:
        GPIO            *_Mux_LEnable_; // GPIO pin for Enabling Demux (Low enable)     [pointer]
        GPIO            *_Mux_HEnable_; // GPIO pin for Enabling Demux (High enable)    [pointer]
        GPIO            *_Mux_A_;       // GPIO pin's for changing output of Demultiplexor
                                        // this is expected to be an array              [pointer]

        uint8_t         _input_size_;   // Contain the number of input switches to Demux

    public:
        DevState        status;         // Indicate the status of the Demultiplexor
        uint8_t         selection;      // Indication of the current selected output
        DevFlt          flt;            // Fault status of the Demultiplexor

        DeMux(GPIO *High_Enable, GPIO *Low_Enable, GPIO Switches[], uint8_t SwitchSize);
        void enable(void);              // Enable the Demultiplexor
        void disable(void);             // Disable the Demultiplexor
        DevFlt updateSelection(uint8_t newselection);    // Update the selection


        virtual ~DeMux();
};

#endif /* DEMUX_H_ */
