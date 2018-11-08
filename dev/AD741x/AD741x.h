/**************************************************************************************************
 * @file        AD741x.h
 * @author      Thomas
 * @version     V0.1
 * @date        08 Nov 2018
 * @brief       Header file for the AD741x series of temperature sensors
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * Class to request and manage data communication between the target embedded device and the
 * selected AD741x temperature sensor device.
 *      Call Class AD741x to initialise class, input needs to detail what the attached device is:
 *          State what device version it is AD7414-0/1/2/3 or AD7415-0/1
 *
 *          Currently both devices are supported, however the additional functionality introduced
 *          within AD7414 has not been captured as of yet.
 *
 *      If in "LiteImplementation" mode, then use of functions like "new"/"delete" are not
 *      supported, therefore the class constructor which supports this is removed from compiler.
 *
 *      Depending upon how the programmer has setup the I2C Device, will change which of the
 *      functions listed below can be used:
 *      For function to wait for new data, or data to be transmitted utilise "poling mode":
 *          ".poleAvailability"     - Determine whether device is available and ready
 *          ".poleConfigRead"       - Read the contents of the device's Configuration Register
 *          ".poleConfigWrite"      - Write new contents to the Configuration Register
 *          ".poleTempRead"         - Read the contents of the device's Temperature Register
 *
 *      If interrupt based communication is to be used then, the following functions are required:
 *          ".intConfigRead"        - Request a read of the contents of device's Configuration
 *                                    Register
 *          ".intConfigWrite"       - Request a write of the device's Configuration Register
 *          ".intTempRead"          - Request a read of the contents of device's Temperature
 *                                    Register
 *
 *          ".intCheckCommStatus"   - Function to be used periodically, so as to determine if
 *                                    interrupt communication has completed AD741x data request.
 *
 *      Fundamental functions used within this class, which are protected, so not visible outside
 *      the class:
 *          ".GetAddress"           - From input parameters on device, retrieve I2C Address (as
 *                                    per Datasheet)
 *          ".AddressForm"          - Used to generate the AD741x Form structure, used to handle
 *                                    what the state of device's Address Pointer has been set too
 *                                    (see section below on AD741x Form)
 *          ".UpdateAddressPointer" - Add a new Address pointer write request to the buffer
 *                                    (only function which puts a "WRITE" state into AD741x form)
 *          ".LastAddresPointRqst"  - Retrieve the last requested/known state of the Address
 *                                    Pointer
 *          ".UpdateConfigReg"      - Construct the byte value to update Configuration Register as
 *                                    per new request
 *          ".DecodeConfig"         - Decode what the read back Configuration value indicates
 *          ".DecodeTempReg"        - Decode what the read back Temperature value is (degC)
 *
 *          ".deconstructData"      - Goes through the input buffer, and based upon the next
 *                                    AD741x form request decode the data
 * 
 *  [#] AD741x Form System
 *      ~~~~~~~~~~~~~~~~~~
 *      The AD741x device utilises a "Address Pointer", which is updated as part of every I2C
 *      write to the device, and is retained until either it is written again or a power cycle
 *      occurs.
 *      So as to manage this, the AD741x class has a internal "Form" system, where every
 *      read/write to the AD741x is captured along with the state of the Address Pointer. This is
 *      then used during the "deconstruct" phase.
 *      The deconstruct phase, will go through this list. Retrieving the state of the Address
 *      Pointer, and updating the class internal pointer state (which in theory should mimic the
 *      Address Pointer within the device). However if the request was for a WRITE, then no
 *      decoding occurs (as no data is read back); but Address Pointer update is retained. If
 *      request was a READ, then the data is decoded.
 *
 *      There is no other functionality within this class
 *************************************************************************************************/
#ifndef AD741X_H_
#define AD741X_H_

#include "FileIndex.h"

#include FilInd_GENBUF_HD               // Provide the template for the circular buffer class
#include FilInd_I2CDe__HD               // Allow use of the I2C Device class

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
// None

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
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
// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
// Address Pointer
// ~~~~~~~~~~~~~~~
#define AD741x_RegSelMASK       0x03                // Mark for Register select bits

#define AD741x_TemperatureReg   0x00                // Point to the Temperature Address
#define AD741x_ConfigReg        0x01                // Point to the Configuration Address
#define AD741x_TempHighReg      0x02                // Point to the Temp High Register (AD7414)
#define AD741x_TempLowReg       0x03                // Point to the Temp Low Register  (AD7414)

// Configuration Register
// ~~~~~~~~~~~~~~~~~~~~~~
#define AD741x_PowerDown        0x80                // Command a power-down of device
#define AD741x_Filter           0x40                // Enable/Disable SDA/SCL filtering
#define AD741x_OneShot          0x04                // Command a one shot temperature conversion

// Temperature Register
// ~~~~~~~~~~~~~~~~~~~~
#define AD741x_TempLowerMask    0xFFC0              // Mask to get lower bits of Temperature value
#define AD741x_TempShift        6                   // Bit position of LSB
#define AD741x_SignBit          0x0200              // Bit position for Negative value

// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
// Defines for the device AD7414
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define AD741x_AD7414_0_Float   (0x0048 << 1)       // I2C Address for device AD7414-0 AS = Float
#define AD741x_AD7414_0_GND     (0x0049 << 1)       // I2C Address for device AD7414-0 AS = GND
#define AD741x_AD7414_0_VDD     (0x004A << 1)       // I2C Address for device AD7414-0 AS = VDD

#define AD741x_AD7414_1_Float   (0x004C << 1)       // I2C Address for device AD7414-1 AS = Float
#define AD741x_AD7414_1_GND     (0x004D << 1)       // I2C Address for device AD7414-1 AS = GND
#define AD741x_AD7414_1_VDD     (0x004E << 1)       // I2C Address for device AD7414-1 AS = VDD
#define AD741x_AD7414_2         (0x004B << 1)       // I2C Address for device AD7414-2 AS = N/A
#define AD741x_AD7414_3         (0x004F << 1)       // I2C Address for device AD7414-3 AS = N/A

// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
// Defines for the device AD7415
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define AD741x_AD7415_0_Float   (0x0048 << 1)       // I2C Address for device AD7415-0 AS = Float
#define AD741x_AD7415_0_GND     (0x0049 << 1)       // I2C Address for device AD7415-0 AS = GND
#define AD741x_AD7415_0_VDD     (0x004A << 1)       // I2C Address for device AD7415-0 AS = VDD
#define AD741x_AD7415_1_Float   (0x004C << 1)       // I2C Address for device AD7415-1 AS = Float
#define AD741x_AD7415_1_GND     (0x004D << 1)       // I2C Address for device AD7415-1 AS = GND
#define AD741x_AD7415_1_VDD     (0x004E << 1)       // I2C Address for device AD7415-1 AS = VDD

// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/

// Types used within this class


class AD741x {
/**************************************************************************************************
 * ==   TYPES   == >>>       TYPES GENERATED WITHIN CLASS        <<<
 *   -----------
 *  Following types are generated within this class. If needed outside of the class, need to
 *  state "AD741x::" followed by the type.
 *************************************************************************************************/
public:
    enum class DevFlt : uint8_t {
        None                = 0x00,
        Fault               = 0x01,
        SizeReqst           = 0x02,
        UnRecognisedAddress = 0x03,
        SynchronizeErr      = 0x04,

        Initialised         = 0xFF
    };

    enum class AddrBit : uint8_t {
        Float       = 0,
        GND         = 1,
        Vdd         = 2
    };

    enum class DevPart : uint8_t {
        AD7414_0        = 0,
        AD7414_1        = 1,
        AD7414_2        = 2,
        AD7414_3        = 3,

        AD7415_0        = 4,
        AD7415_1        = 5
    };

    enum class PwrState : uint8_t {FullPower, StandBy};

    enum class FiltState: uint8_t {Enabled, Disabled};

    enum class OneShot  : uint8_t {TrigConv, Nothing};

    typedef struct {
        uint8_t     AddrPoint;
        enum Dir : uint8_t {Write, Read} ReadWrite;
    } Form;

/**************************************************************************************************
 * == GEN PARAM == >>>       GENERIC PARAMETERS FOR CLASS        <<<
 *   -----------
 *  Parameters required for the class to function.
 *************************************************************************************************/
    protected:
        DevPart     PartNumber;     // Retain the selected part number of device
        AddrBit     AddressPin;     // Retain the status of the Address Pin
        PwrState    Mode;           // Mode of the device
        FiltState   FiltMode;       // Mode of the device filter

        uint16_t    I2CAddress;     // I2C Address of the device
        uint8_t     AddressPointer; // Stores the current state of the Address Pointer
        GenBuffer<uint8_t>  *rdBuff;// Read buffer for AD741x device communications
        GenBuffer<Form>     *AdBuff;// Buffer for the state of the Address pointer, needs to line
                                    // up with any read backs (as this indicates what has been
                                    // read!)
        GenBuffer<uint8_t>  *wtBuff;// Write buffer for AD741x device communications

    public:
        DevFlt      Flt;            // Fault status of the device

        float       Temp;           // Read temperature from device (degC)
        int16_t     TempReg;        // Read temperature register

        // Parameters used for interrupt based I2C communication
        I2CDevice::DevFlt   I2CWFlt;// Fault status of the I2C write Communication
        uint8_t     wtcmpFlag;      // I2C write communication complete flag
        uint8_t     wtcmpTarget;    // I2C write communication number of bytes to have been
                                    // transmitted

        I2CDevice::DevFlt   I2CRFlt;// Fault status of the I2C read Communication
        uint8_t     rdcmpFlag;      // I2C read communication complete flag
        uint8_t     rdcmpTarget;    // I2C read communication number of bytes to have been
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
    protected:
        void popGenParam(DevPart DeviceNum, AddrBit ASPin);     // Populate generic parameters for
                                                                // class generation

    public:
        AD741x(DevPart DeviceNum, AddrBit ASPin, GenBuffer<Form> *AddressBuffer,
                GenBuffer<uint8_t> *readBuffer, GenBuffer<uint8_t> *writeBuffer);


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#ifndef __LiteImplement__       // If "__LiteImplement__" has not been defined, then allow use of
                                // "new" and "delete" for defining internal arrays
                                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        AD741x(DevPart DeviceNum, AddrBit ASPin);       // Basic constructor for class, the
                                                        // internal buffers are set to default
                                                        // size

        AD741x(DevPart DeviceNum, AddrBit ASPin, uint32_t AddBuffSize, uint32_t CommBuffSize);
        // Basic constructore for the class, where the sizes of both the internal Address and
        // communication buffers has been provided. These buffers will be constructed within
        // this function call

#endif                          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/**************************************************************************************************
 * == GEN FUNCT == >>>      GENERIC FUNCTIONS WITHIN CLASS       <<<
 *   -----------
 *  The following are functions scoped within the "AD741x" class, which are generic; this means
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
    void GetAddress(void);          // Determine the Address from provided device number
    Form AddressForm(uint8_t newAdd, Form::Dir Direction);

    uint8_t UpdateAddressPointer(GenBuffer<uint8_t> *buff, uint8_t newval);
    // Generate write request to update Address Pointer in AD741x device
    uint8_t LastAddresPointRqst(void);              // Retrieve the last Address Pointer request

    uint8_t UpdateConfigReg(GenBuffer<uint8_t> *buff,
                            PwrState Mode, FiltState Filt, OneShot Conv);
    // Generate write request to update the Configuration Register

    void DecodeConfig(uint8_t data);                // Decode the Configuration Register
    void DecodeTempReg(uint8_t *pData);             // Decode the Temperature Registers

    DevFlt deconstructData(GenBuffer<uint8_t> *readData, uint16_t size);
            // Go through input "GenBuffer" read data, and only deconstruct "size" entries.

public:     /**************************************************************************************
             * ==  PUBLIC   == >>>    POLING FUNCTIONS FOR DATA TRANSFER     <<<
             *   -----------
             *  Visible functions used to setup and transfer packets of data to the selected
             *  AD741x device - Will wait for any hardware registers to be in correct state before
             *  progressing.
             *************************************************************************************/
    DevFlt poleAvailability(I2CDevice *hal_I2C);            // Check to see if device is available
    DevFlt poleConfigRead(I2CDevice *hal_I2C);              // Read Configuration of Device
    DevFlt poleConfigWrite(I2CDevice *hal_I2C,
                                      PwrState Mode, FiltState Filt, OneShot Conv);
        // Update the contents of the Configuration register as per input conditions

    DevFlt poleTempRead(I2CDevice *hal_I2C);                // Read the contents of the
                                                            // temperature register

public:     /**************************************************************************************
             * ==  PUBLIC   == >>>   INTERRUPT FUNCTIONS FOR DATA TRANSFER   <<<
             *
             *  Visible functions used to support communication to AD741x device, on a interrupt
             *  bases.
             *      Enabling of interrupt bits/DMAs are not handled within this class.
             *************************************************************************************/
    void intConfigRead(I2CDevice *hal_I2C);                 // Request read of Configuration
                                                            // Register
    void intConfigWrite(I2CDevice *hal_I2C,
                        PwrState Mode, FiltState Filt, OneShot Conv);
        // Request write to update contents of the Configuration Register

    void intTempRead(I2CDevice *hal_I2C);                   // Request temperature read

    void intCheckCommStatus(void);                          // Check to see if interrupt based
                                                            // communication has completed
                                                            // Will then update class parameters
    virtual ~AD741x();
};

#endif /* AD741X_H_ */
