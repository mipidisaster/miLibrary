/**************************************************************************************************
 * @file        DeMux.cpp
 * @author      Thomas
 * @brief       Source file for the Demultiplexor Class handle
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include <FileIndex.h>                  // Header for miLibrary index
#include FilInd_DeMux__HD               // Header for DeMux

// C System Header(s)
// ------------------
#include <stdint.h>

// C++ System Header(s)
// --------------------

// Other Libraries
// --------------
#if   (zz__MiEmbedType__zz ==  50)      // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
// None

#elif (zz__MiEmbedType__zz ==  51)      // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
// None

#elif (zz__MiEmbedType__zz ==  10)      // If the target device is an Raspberry Pi then
//=================================================================================================
// None

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
#include FilInd_GPIO___HD               // Defines class - GPIO

//=================================================================================================

DeMux::DeMux(GPIO *High_Enable, GPIO *Low_Enable, GPIO Switches[], uint8_t SwitchSize) {
/**************************************************************************************************
 * Constructor function for the Demultiplexor class.
 * It assumes that the High_Enable and Low_Enable GPIOs are single entries only (so if provided
 * an array, will only set the first entry)
 * Switches is the only GPIO which is expected to be an array (hence why call is using "[]")
 * The number of switches needs to be provided to the class - should be the same as the number of
 * entries within the Switches array
 *
 * Entry [0] will be taken as the LSB, Entry [n] will be taken as MSB.
 *  Where "n" is the size of the array
 *
 * As some Demultiplexors may not use Enable switches, if they are not required - or managed
 * outside of this class, then provide "__null", and they will be ignored
 *
 * Before completing construction of class, it will disable the Demultiplexor, by setting all
 * enabling conditions "OFF".
 *************************************************************************************************/
    _Mux_HEnable_   = High_Enable;      // Pass the High ENABLE GPIO
    _Mux_LEnable_   = Low_Enable;       // Pass the Low ENABLE GPIO
    _Mux_A_         = Switches;         // Pass ARRAY of switches

    _input_size_     = SwitchSize;      // Provide the number of switches to be controlled which
                                        // is the size of the array

    selection     = -1;                 // Provide a Selection, however as just setup class should
                                        // be an unrecognised entry "-1"
    flt           = DevFlt::kInitialised;   // Set the fault status to "Initialised"

    // Now that have populated the DeMux class, Disable the DeMux
    disable();    // Call the disable function
}

void DeMux::enable(void) {
/**************************************************************************************************
 * Simple function, for enabling the Demultiplexor
 * How the function does this depends upon which enabling conditions have been provided to the
 * class.
 * If any of the High/Low Enable GPIOs have been provided then it will set these accordingly:
 *  High Enable (HEnable)   = Will be set High
 *  Low Enable (LEnable)    = will be set Low
 *
 * If any of these have not been defined (i.e. __null will have been provided), then the function
 * will skip them.
 *************************************************************************************************/
    if (_Mux_HEnable_ != __null) {              // If a High Enable pin has been defined then
        _Mux_HEnable_->setValue(GPIO::kHigh);   // Set the Pin HIGH to enable DeMux
    }

    if (_Mux_LEnable_ != __null) {              // If a Low Enable pin has been defined then
        _Mux_LEnable_->setValue(GPIO::kLow);    // Set the Pin LOW to enable DeMux
    }
    status = DevState::kEnable;                 // Update status to "Enabled"
}

void DeMux::disable(void) {
/**************************************************************************************************
 * Simple function, for disabling the Demultiplexor
 * How the function does this depends upon which disabling conditions have been provided to the
 * class.
 * If any of the High/Low Enable GPIOs have been provided then it will set these accordingly:
 *  High Enable (HEnable)   = Will be set Low
 *  Low Enable (LEnable)    = will be set High
 *
 * If any of these have not been defined (i.e. __null will have been provided), then the function
 * will skip them.
 *************************************************************************************************/
    if (_Mux_HEnable_ != __null) {              // If a High Enable pin has been defined then
        _Mux_HEnable_->setValue(GPIO::kLow);    // Set the Pin LOW to disable DeMux
    }

    if (_Mux_LEnable_ != __null) {              // If a Low Enable pin has been defined then
        _Mux_LEnable_->setValue(GPIO::kHigh);   // Set the Pin HIGH to disable DeMux
    }
    status = DevState::kDisable;               // Update status to "Disabled"
}
DeMux::DevFlt DeMux::updateSelection(uint8_t newselection)
{
    GPIO::setGPIOArray(_Mux_A_, _input_size_, newselection);

    flt = DevFlt::kNone;    // If have gotten this far, then update has worked - Updated status
    return(flt);            // Provide positive return
}

DeMux::~DeMux()
{
    // TODO Auto-generated destructor stub
}

