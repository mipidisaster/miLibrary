/**************************************************************************************************
 * @file        DeMux.cpp
 * @author      Thomas
 * @version     V2.1
 * @date        21 Dec 2018
 * @brief       Source file for the Demultiplexor Class handle
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include "FileIndex.h"
#include FilInd_DeMux__HD

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
    this->Mux_HEnable   = High_Enable;      // Pass the High ENABLE GPIO
    this->Mux_LEnable   = Low_Enable;       // Pass the Low ENABLE GPIO
    this->Mux_A         = Switches;         // Pass ARRAY of switches

    this->inputsize     = SwitchSize;       // Provide the number of switches to be controlled
                                            // which is the size of the array

    this->Selection     = -1;               // Provide a Selection, however as just setup class
                                            // should be an unrecognised entry "-1"
    this->Flt           = DeMux_Initialised;// Set the fault status to "Initialised"

    // Now that have populated the DeMux class, Disable the DeMux
    this->disable();    // Call the disable function
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
    if (this->Mux_HEnable != __null) {          // If a High Enable pin has been defined then
        this->Mux_HEnable->setValue(GPIO::HIGH);// Set the Pin HIGH to enable DeMux
    }

    if (this->Mux_LEnable != __null) {          // If a Low Enable pin has been defined then
        this->Mux_LEnable->setValue(GPIO::LOW); // Set the Pin LOW to enable DeMux
    }
    this->Status = DeMux_Enabled;               // Update status to "Enabled"
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
    if (this->Mux_HEnable != __null) {         // If a High Enable pin has been defined then
        this->Mux_HEnable->setValue(GPIO::LOW);// Set the Pin LOW to disable DeMux
    }

    if (this->Mux_LEnable != __null) {          // If a Low Enable pin has been defined then
        this->Mux_LEnable->setValue(GPIO::HIGH);// Set the Pin HIGH to disable DeMux
    }
    this->Status = DeMux_Disabled;              // Update status to "Disabled"
}
_DeMuxFlt DeMux::updateselection(uint8_t newselection)
{
    uint8_t temp = 0;       // Temporary variable used within this function, for
                            //  -> Determining maximum size of Demultiplexor selection
                            //  -> Looping through input selection, to set corresponding switch

    temp = ((1 << this->inputsize) - 1);    // Calculate the maximum size, by shifting 0x01 up by
                                            // number of switches provided. Then subtracting 1
                                            //      equivalent to (2^x) - 1.

    if (newselection > temp) {                  // If the selection is greater than the number of
                                                // switches can accommodate
        this->Flt = DeMux_IncorrectSelection;   // Indicate fault with selection
        return (this->Flt);                     // Return fault code
    }

    // If get to this point, then input selection is within capabilitys of the class setup
    for (temp = 0; temp != (this->inputsize); temp++) {     // Now use temporary variable to loop
        if (temp != 0) {                        // If not the first pass of loop then
            newselection >>= 1;                 // binary shift the input selection number by 1
        }                                       // to the right (make it smaller)

        if (newselection & 1)                       // If lowest bit is "1", then
            this->Mux_A[temp].setValue(GPIO::HIGH); // Set corresponding switch "HIGH"
        else                                        // If lowest bit is "0", then
            this->Mux_A[temp].setValue(GPIO::LOW);  // Set corresponding switch "LOW"

    }

    this->Flt = DeMux_NoFault;  // If have gotten this far, then update has worked - Updated status
    return(this->Flt);          // Provide positive return
}

DeMux::~DeMux()
{
    // TODO Auto-generated destructor stub
}

