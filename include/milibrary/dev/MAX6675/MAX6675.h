/**************************************************************************************************
 * @file        MAX6675.h
 * @author      Thomas
 * @brief       Header file for the MAX6675 Thermocouple Digital Converter
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * Class to request and manage data communication between the target embedded device and the
 * MAX6675 temperature sensor.
 *      Call Class MAX6675 to initialise class, detail connected chip selection
 *      Currently supports
 *          Hardware Chip Select
 *          Individual GPIO
 *          DeMux (Support only for 'pole' mode, 'int' mode not supported see issue #12 in github)
 *
 *      Depending upon how the programmer has setup the SPI Device, will change which of the
 *      functions listed below can be used:
 *      For function to wait for new data, or data to be transmitted utilise "poling mode":
 *          ".poleTempRead"         - Read the data from MAX6675, and decode
 *
 *      If interrupt based communication is to be used then, the following functions are required:
 *          ".reInitialise"         - Resets the internal mechanics of the class
 *          ".intTempRead"          - Request a read of the contents of MAX6675
 *          ".intCheckCommState"    - Function to be used periodically, after confirming that
 *                                    the interrupt SPI routine has completed communication.
 *
 *      Fundamental functions used within this class, which are protected, so not visible outside
 *      the class.
 *          ".decodeMAX6675"        - Decode the read data from device, and calculate the
 *                                    temperature.
 *
 *      There is no other functionality within this class
 *************************************************************************************************/
#ifndef MAX6675_H_
#define MAX6675_H_

#include "FileIndex.h"
// C System Header(s)
// ------------------
#include <stdint.h>

// C++ System Header(s)
// --------------------
// None

// Other Libraries
// --------------
#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
// None

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
// None

#elif (zz__MiEmbedType__zz == 10)       // If the target device is an Raspberry Pi then
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
#include FilInd_GPIO___HD               // Allow use of GPIO class, for Chip Select
#include FilInd_DeMux__HD               // Allow use of DeMux class, for Chip Select
#include FilInd_SPIPe__HD               // Include class for SPI Peripheral

//=================================================================================================

// Defines specific within this class
// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
#define MAX6675_Dummy       0x8000      // Bit 15 set TRUE - Dummy bit incorrect
#define MAX6675_NoThermo    0x0004      // Bit 2  set TRUE - Indicates open circuit on thermocouple
#define MAX6675_DevID       0x0002      // Bit 1  set TRUE - Device ID incorrect

#define MAX6675_Temp        0x7FF8      // 12bit position for the temperature

// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/

class MAX6675 {
/**************************************************************************************************
 * ==   TYPES   == >>>       TYPES GENERATED WITHIN CLASS        <<<
 *   -----------
 *  Following types are generated within this class. If needed outside of the class, need to
 *  state "MAX6675::" followed by the type.
 *************************************************************************************************/
public:
    enum class DevFlt : uint8_t {
        kNone               = 0x00,
        kRead_Error         = 0x01,
        kDummy_Fault        = 0x02,
        kDevice_ID_Fault    = 0x03,
        kNo_Sensor          = 0x04,
        kInitialised        = 0xFF
    };

/**************************************************************************************************
 * == GEN PARAM == >>>       GENERIC PARAMETERS FOR CLASS        <<<
 *   -----------
 *  Parameters required for the class to function.
 *************************************************************************************************/
protected:
    GPIO        *_gpio_select_;         // Optional GPIO class for Chip Select

    DeMux       *_demux_select_;        // Optional DeMux class for Chip Select
    uint8_t     _demux_select_number_;  // Optional DeMux selection number

    uint8_t     _device_contents_[2];   // 2d array to contain the most recent read from the
                                        // selected device.

    void popGenParam(void);             // Populate generic parameters for the class

public:
    float       temp;           // Calculated temperature (celsius) from last "readTemp"
    DevFlt      flt;            // Fault indication of Temperature from last "readTemp"
    uint16_t    raw_data;       // Raw data read from MAX6675

    // Parameters used for interrupt based SPI communication
    volatile SPIPeriph::DevFlt  spi_read_flt;   // Fault status of the SPI read communication
    volatile uint16_t    read_cmp_flag;         // SPI write communication complete flag
    uint16_t    read_cmp_target;                // SPI read communication number of bytes to have
                                                // been received

/**************************************************************************************************
 * == SPC PARAM == >>>        SPECIFIC ENTRIES FOR CLASS         <<<
 *   -----------
 *  Following are functions and parameters which are specific for the embedded device selected.
 *  The initialisation function for the class is also within this section, which again will be
 *  different depending upon the embedded device selected.
 *  >> NOTE <<
 *      As this is a class which utilises the lower level classes, the selection of the embedded
 *      device at this level doesn't change how the class works, therefore there is no selection
 *      of different devices.
 *************************************************************************************************/
public:
    MAX6675();
    MAX6675(GPIO *ChipSelect);
    MAX6675(DeMux *DeMuxCS, uint8_t CSNum);
    // Setup the MAX6675 device
    // OVERLOADED function, with variations such that the interface to the MAX6675 can be
    // controlled, either by hardwared SPI (no function arguments), individually controlled GPIO,
    // or DeMux configuration

    virtual ~MAX6675();

/**************************************************************************************************
 * == GEN FUNCT == >>>      GENERIC FUNCTIONS WITHIN CLASS       <<<
 *   -----------
 *  The following are functions scoped within the "AS5x4x" class, which are generic; this means
 *  are used by ANY of the embedded devices supported by this class.
 *  >> NOTE <<
 *      As this is a class which utilises the lower level classes, the selection of the embedded
 *      device at this level doesn't change how the class works, therefore there is no selection
 *      of different devices.
 *************************************************************************************************/
protected:  /**************************************************************************************
             * == PROTECTED == >>>  FUNDAMENTAL FUNCTION FOR CLASS TO WORK   <<<
             *   -----------
             *  These functions are the bases for use of this class. The "upper level" function
             *  (which are public), rely upon these functions to operate.
             *  Are protected, as the upper level functions will not need to use these.
             *************************************************************************************/
    DevFlt decodeMAX6675(void);     // Function to take the internal array '_device_contents_', and
                                    // determine state of device/temperature

public:     /**************************************************************************************
             * ==  PUBLIC   == >>>    POLING FUNCTIONS FOR DATA TRANSFER     <<<
             *   -----------
             *  Visible functions used to setup and transfer packets of data to the selected
             *  AS5x4x device - Will wait for any hardware registers to be in correct state before
             *  progressing.
             *************************************************************************************/
    void        poleTempRead(SPIPeriph *Interface);

public:     /**************************************************************************************
             * ==  PUBLIC   == >>>   INTERRUPT FUNCTIONS FOR DATA TRANSFER   <<<
             *
             *  Visible functions used to support communication to AD741x device, on a interrupt
             *  bases.
             *      Enabling of interrupt bits/DMAs are not handled within this class.
             *************************************************************************************/
    void        reInitialise(void);
    void        intTempRead(SPIPeriph *Interface);
    void        intCheckCommStatus(void);

};

#endif /* MAX6675_H_ */
