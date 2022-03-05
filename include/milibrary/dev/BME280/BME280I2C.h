/**************************************************************************************************
 * @file        BME280I2C.h
 * @author      Thomas
 * @brief       Header file for the BMR280 series of temperature sensors (I2C interface)
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * Class to request and manage data communication between the target embedded device and the
 * selected BME280 "Combined humidity and pressure sensor".
 *      Call Class "BME280I2C" to initialise class, inputs needs to detail what the state of the
 *          Address bit is of the target BME280 device.
 * 
 * This class is based upon the "BME280" class, and introduces the I2C interface functionality.
 * The changes that this class introduces relative to the "BME280" class:
 *      Protected function(s) -
 *          ".getI2CAddress"        - Based upon the Address bit provided at class construction
 *                                    will determine what the I2C address is
 *
 *      For function to wait for new data, or data to be transmitted utilise "poling mode":
 *          ".poleAvailability"     - Determine whether device is available and read
 *          ".poleCalibrateRead"    - Request a read of the Calibration/Conversion register(s)
 *          ".poleConfigRead"       - Request a read of the Configuration register(s)
 *          ".poleConfigWrite"      - Request a write of the Configuration register(s)
 *          ".poleSensorRead"       - Read the contents of the devices Sensor read Register(s)
 *                                    note - Will also convert to actual
 *                                           Temperature/Pressure/Humidity
 *
 *      If interrupt based communication is to be used then, the following functions are required:
 *          ".reInitialise"         - Resets the internals mechanics of the class
 *          ".intCalibrateRead"     - Request a read of the Calibration/Conversion register(s)
 *          ".intConfigRead"        - Request a read of the Configuration register(s)
 *          ".intConfigWrite"       - Request a write of the Configuration register(s)
 *          ".intSensorRead"        - Read the contents of the devices Sensor read Register(s)
 * 
 *          ".clearCommunicationCount"
 *                                  - Clear the internal flags within class for interrupt
 *                                    communication management
 *          ".intCheckCommStatus"   - Function to be used periodically, after confirming that the
 *                                    interrupt I2C routine has completed communication
 *          ".getI2CAddress"        - Return the target I2C address
 *
 *  [#] Recommended Settings
 *      ~~~~~~~~~~~~~~~~~~~~
 *      The datasheet for the BME280 provides recommended settings for the device. These settings
 *      have been baked into the contents of this class, and are able to be setup via:
 *          ".pole"/".intRecommendedConfigWeather"
 *          ".pole"/".intRecommendedConfigHumidity"
 *          ".pole"/".intRecommendedConfigIndoor"
 *          ".pole"/".intRecommendedConfigGaming"
 *
 *************************************************************************************************/
#ifndef BME280I2C_H_
#define BME280I2C_H_

#include "FileIndex.h"
// C System Header(s)
// ------------------
#include <stdint.h>

// C++ System Header(s)
// --------------------

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
#include FilInd_BME280_HD               // Provide the lower level class for the BME280 device
#include FilInd_I2CPe__HD               // Include class for I2C Peripheral

//=================================================================================================

// Defines specific within this class
// Defined in the "BME280_defs.h" file
// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/

class BME280I2C  : public BME280 {
/**************************************************************************************************
 * ==   TYPES   == >>>       TYPES GENERATED WITHIN CLASS        <<<
 *   -----------
 *  Following types are generated within this class. If needed outside of the class, need to
 *  state "BME280I2C::" followed by the type.
 *************************************************************************************************/
    public:
        enum class AddrBit : uint8_t {
            kFloat      = 0,
            kGnd        = 1,
            kVdd        = 2
        };

/**************************************************************************************************
 * == GEN PARAM == >>>       GENERIC PARAMETERS FOR CLASS        <<<
 *   -----------
 *  Parameters required for the class to function.
 *************************************************************************************************/
    protected:
        AddrBit     _address_pin_;      // Retain the status of the Address Pin

        uint16_t    _i2c_address_;      // I2C Address of the device

    public:
        // Parameters used for interrupt based I2C communication
        volatile I2CPeriph::DevFlt   i2c_wrte_flt;  // Fault status of the I2C write Communication
        volatile uint16_t    wrte_cmp_flg;          // I2C write communication complete flag
        uint16_t    wrte_cmp_target;    // I2C write communication number of bytes to have been
                                        // transmitted

        volatile I2CPeriph::DevFlt   i2c_read_flt;  // Fault status of the I2C read Communication
        volatile uint16_t    read_cmp_flag;         // I2C read communication complete flag
        uint16_t    read_cmp_target;    // I2C read communication number of bytes to have been
                                        // received

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
    BME280I2C(AddrBit ASPin, Form *FormArray, uint16_t FormSize);

    virtual ~BME280I2C();

/**************************************************************************************************
 * == GEN FUNCT == >>>      GENERIC FUNCTIONS WITHIN CLASS       <<<
 *   -----------
 *  The following are functions scoped within the "BME280I2C" class, which are generic; this means
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
    void getI2CAddress(void);       // Determine the Address from provided device number

public:     /**************************************************************************************
             * ==  PUBLIC   == >>>    POLING FUNCTIONS FOR DATA TRANSFER     <<<
             *   -----------
             *  Visible functions used to setup and transfer packets of data to the selected
             *  BME280 device - Will wait for any hardware registers to be in correct state before
             *  progressing.
             *************************************************************************************/
    DevFlt poleAvailability (I2CPeriph *Interface);
    DevFlt poleCalibrateRead(I2CPeriph *Interface);
    DevFlt poleConfigRead   (I2CPeriph *Interface);
    DevFlt poleConfigWrite  (I2CPeriph *Interface,
                               ctrl_hum humd_config, osrs_p pres_config, osrs_t temp_config,
                               ctrl_mode sensor_mode, t_sb t_standby, cfig_filt filter_constant);

    DevFlt poleSensorRead   (I2CPeriph * Interface);

    // Recommended configurations - see Datasheet section 3.5
    DevFlt poleRecommendedConfigWeather (I2CPeriph *Interface);
    DevFlt poleRecommendedConfigHumidity(I2CPeriph *Interface);
    DevFlt poleRecommendedConfigIndoor  (I2CPeriph *Interface);
    DevFlt poleRecommendedConfigGaming  (I2CPeriph *Interface);

public:     /**************************************************************************************
             * ==  PUBLIC   == >>>   INTERRUPT FUNCTIONS FOR DATA TRANSFER   <<<
             *
             *  Visible functions used to support communication to BME280 device, on a interrupt
             *  bases.
             *      Enabling of interrupt bits/DMAs are not handled within this class.
             *************************************************************************************/
    void reInitialise(void);                                // Initialise the internal Address
                                                            // Pointer, etc.
    void intCalibrateRead   (I2CPeriph *Interface, uint8_t *rBuff, uint8_t *wBuff);
    void intConfigRead      (I2CPeriph *Interface, uint8_t *rBuff, uint8_t *wBuff);
    void intConfigWrite     (I2CPeriph *Interface, uint8_t *wBuff,
                               ctrl_hum humd_config, osrs_p pres_config, osrs_t temp_config,
                               ctrl_mode sensor_mode, t_sb t_standby, cfig_filt filter_constant);

    void intSensorRead      (I2CPeriph *Interface, uint8_t *rBuff, uint8_t *wBuff);

    // Recommended configurations - see Datasheet section 3.5
    void intRecommendedConfigWeather (I2CPeriph *Interface, uint8_t *wBuff);
    void intRecommendedConfigHumidity(I2CPeriph *Interface, uint8_t *wBuff);
    void intRecommendedConfigIndoor  (I2CPeriph *Interface, uint8_t *wBuff);
    void intRecommendedConfigGaming  (I2CPeriph *Interface, uint8_t *wBuff);

    uint8_t intCheckCommStatus(uint8_t *rBuff);
    // Will take the input parameters and decode the specified number of entries
    void clearCommunicationCount(void);

    uint16_t readI2CAddress(void);
};

#endif /* BME280I2C_H_ */
