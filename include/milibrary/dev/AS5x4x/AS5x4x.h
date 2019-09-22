/**************************************************************************************************
 * @file        AS5x4x.h
 * @author      Thomas
 * @version     V3.4
 * @date        22 Sept 2019
 * @brief       Header file for the AMS Angular Position device (AS5x4x)
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * Class to request and manage data communications between the target embedded device and the
 * selected AS5x4x device.
 *      Call Class AS5x4x to initialise class, detail the attached device.
 *      Currently only supports:
 *          AS5047D
 *          AS5048A
 *
 *      If in "LiteImplementation" mode, then pointers to the write and read buffers need to be
 *      provided. Otherwise they will be generated internally to the class.
 *
 *      Depending upon how the programmer has setup the SPI Device, will change which of the
 *      functions listed below can be used:
 *      For function to wait for new data, or data to be transmitted utilising "poling mode":
 *          ".poleSPITransmit"      - This is an OVERLOADED function, where only the SPI + Chip
 *                                    select is provided, then function will communication with
 *                                    only the specific class AS5x4x (assumes device is directly
 *                                    linked)
 *                                    Other configuration is to include the "Daisy" structure (see
 *                                    below), which will transmit to multiple devices
 *
 *      If interrupt based communication is to be used then, the following functions are required:
 *          ".reInitialise"         - Initialise the internal forms/queues in fault situations.
 *          ".intSingleTransmit"    - OVERLADED function, similar to the "poling" version. Either
 *                                    provide it just the SPI + CS, or Daisy chain and SPI
 *
 *      Generic functions used to build/manage the device(s) either in a Daisy format (see below):
 * static   ".SPIReadChain"         - Read any SPI read data, and transfer to internal class 16bit
 *                                    queue, for later deconstruction.
 *          ".checkDataRequest"     - See if the internal write buffer contains any new requests
 *                                    to be transfered to device
 *          ".readDataPacket"       - Scan through all unread internal 16bit registers and
 *                                    deconstruct as per device format, and make available
 *                                    external to class
 * static   ".constructDaisy"       - Construct a Daisy change structure, with "x" number of
 *                                    AS5x4x devices (1st entry is last in the actual DAISY)
 * static   ".checkDaisyRequest"    - Scan through all devices within Daisy chain and check for
 *                                    any new requests
 * static   ".readDaisyPackets"     - Similar to "readDataPacket", however is done for all devices
 *                                    attached in Daisy chain
 *
 *      Fundamental functions used within the class, which are protected, so not visible outside
 *      of class:
 * static   ".EventParityCheck"     - Check data packet to see if parity is correct
 * static   ".SPIWriteChain"        - Baseline function to transmit data in poling mode
 *          ".deconstructAS5048A"   - Deconstruct the AS5048A data from device
 *          ".deconstructAS5047D"   - Deconstruct the AS5047D data from device
 *
 *      Functions to requested data from the AS5x4x device:
 *          constructNOP, constructCEF, constructAGC, constructMag, constructAng
 *          constructALL
 *
 *  [#] AS5x4x Daisy Struct
 *      ~~~~~~~~~~~~~~~~~~~
 *      As the AS5x4x devices all support use of devices in a "Daisy Chain" format - see datasheet
 *      for details. This class utilises a "Daisy" structure, which can be constructed (and is
 *      suggested to be used anyway for interrupt based communication - irrespective on number of
 *      devices in chain).
 *      This structure contains a pointer to the array of constructed AS5x4x devices, along with
 *      the number of devices. Also contains parameters used to manage interrupt based
 *      communication, namely - Fault status, Target number of transfers, Complete flag.
 *
 *      There is no other functionality within this class
 *************************************************************************************************/
#ifndef AS5X4X_H_
#define AS5X4X_H_

#include "FileIndex.h"
#include <stdint.h>

#include FilInd_GENBUF_HD               // Provide the template for the circular buffer class
#include FilInd_GPIO___HD               // Allow use of GPIO class, for Chip Select
#include FilInd_SPIPe__HD               // Include class for SPI Peripheral

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
// Generic Defines for AS5x4x devices
// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
// > SPI Communication Command Packages
#define AS5x4x_PARITY   0x8000          // Position for EVEN Parity bit
#define AS5x4x_ReadMask 0x4000          // Position for the Write/Read bit (0 / 1 respectively)
#define AS5x4x_DataMask 0x3FFF          // Mask to get the address sent only
// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
// > Error Register
#define AS5x4x_ErrorMask                0x0007      // Mask for getting device errors
#define AS5x4x_FrameError               0x0001      // Mask for Framing error
#define AS5x4x_CmdIvError               0x0002      // Mask for Command Invalid
#define AS5x4x_ParitError               0x0004      // Mask for Parity error

// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
// > AGC Register Mask
#define AS5x4x_AGVValueMask             0x00FF      // Mask to get the Automatic Gain Control value
#define AS5x4x_DiagnFlags               0x0F00      // Mask to get the Diagnostic flags
#define AS5x4x_OCFMask                  0x0001      // Mask to get the Offset Compensation
                                                    // Algorithm flag
#define AS5x4x_COFMask                  0x0002      // Mask to get the CORDIC Overflow flag
#define AS5x4x_CompLow                  0x0004      // Mask to get the Comp Low flag
#define AS5x4x_CompHigh                 0x0008      // Mask to get the Comp High flag

// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
// Defines for the device AS5x47
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define AS5047_NOP      0x0000          // Address for No operation message
#define AS5047_ERRFL    0x0001          // Address for the Error Register
#define AS5047_PROG     0x0003          // Address for the Programming Register
#define AS5047_DIAAGC   0x3FFC          // Address for the Diagnostic and AGC Register
#define AS5047_MAG      0x3FFD          // Address for the CORDIC magnitude
#define AS5047_ANGLE    0x3FFF          // Address for the Measured angle with dynamic angle error
// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
// Defines for the device AS5x48
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define AS5048_NOP      0x0000          // Address for No operation message
#define AS5048_ERRFL    0x0001          // Address for the Error Register
#define AS5048_PROG     0x0003          // Address for the Programming Register
#define AS5048_DIAAGC   0x3FFD          // Address for the Diagnostic and AGC Register
#define AS5048_MAG      0x3FFE          // Address for the CORDIC magnitude
#define AS5048_ANGLE    0x3FFF          // Address for the Measured angle
// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/

#define AS5x4x_MAXChain     10          // Define the maximum number of AS5x4x devices in a chain

#ifndef PI
#define PI                 3.14159265358979f
#endif

// Types used within this class


class AS5x4x {
/**************************************************************************************************
 * ==   TYPES   == >>>       TYPES GENERATED WITHIN CLASS        <<<
 *   -----------
 *  Following types are generated within this class. If needed outside of the class, need to
 *  state "AS5x4x::" followed by the type.
 *************************************************************************************************/
public:
    enum class DevFlt : uint8_t {
        None                = 0x00,
        Frame               = 0x01,
        CmdInv              = 0x02,
        Parity              = 0x03,
        MultiFault          = 0x04,

        NoCommunication     = 0xE0,
        ParentCommFlt       = 0xF0,

        Initialised         = 0xFF
    };

    enum class DevPart : uint8_t {
        AS5048A         = 0,
        AS5047D         = 1
    };

    typedef struct {                    // Structure used to handle multiple AS5x4x devices in a
                                        // Daisy Chain format
                                        // Consult the AS5048A datasheet to see how this needs to
                                        // constructed.
        AS5x4x          *Devices;       // Pointer to the devices in the chain, note the first
                                        // entry, needs to be the LAST in the chain!
        uint16_t        numDevices;     // Number of devices

        volatile SPIPeriph::DevFlt  Flt;    // Pointer to fault flag for interrupt based
                                            // communication
        volatile uint16_t           Cmplt;  // Pointer to completed flag for interrupt based
                                            // communication
        uint16_t                    Trgt;   // SPI target number of bytes to be requested to be
                                            // transfered.
    }   Daisy;

/**************************************************************************************************
 * == GEN PARAM == >>>       GENERIC PARAMETERS FOR CLASS        <<<
 *   -----------
 *  Parameters required for the class to function.
 *************************************************************************************************/
protected:
    GenBuffer<uint16_t> rdBuff;         // Read buffer for data transfer
    GenBuffer<uint16_t> wtBuff;         // Write buffer for data transfer

    void popGenParam(void);             // Populate generic parameters for the class

public:
    float       Angle;                  // Calculated real angle from device (radians)
    uint16_t    AngularSteps;           // Read number of Angular steps (0 .. 16384)
    uint16_t    Mag;                    // Magnitude of Magnetic flux from CORDIC
                                        // CORDIC = Coordinate Rotation Digital Computer
    uint16_t    AGC;                    // Automatic Gain Control value
    uint16_t    Diagnostic;             // Diagnostic flags
    DevFlt      Flt;                    // Fault status of the device

    DevPart     Device;                 // Store the device type

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
    AS5x4x(DevPart Device, uint16_t *wtBuff, uint16_t *rdBuff, uint16_t size);
    // Setup the AS5x4x device, by providing the device type, as well as array pointer(s) to be
    // used as the internal write and read buffers.

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
    static uint8_t EvenParityCheck(uint16_t packet);    // Determine if the data packet is even
                                                        // parity
    void WriteDataPacket(uint16_t PacketData);          // Check data for parity, and put into
                                                        // Buffer

    static uint16_t SPIWriteChain(AS5x4x *device, uint16_t numchain, uint8_t *wtdata);

    void deconstructAS5048A(uint16_t Address, uint16_t packetdata); // Deconstruct AS5048A data
    void deconstructAS5047D(uint16_t Address, uint16_t packetdata); // Deconstruct AS5047D data

public:     /**************************************************************************************
             * ==  PUBLIC   == >>>    GENERIC FUNCTIONS FOR DATA REQUESTS    <<<
             *   -----------
             *  Visible functions which are generic for any use of the SPI peripheral.
             *  Are used to build up a "queue" of requests on the selected AS5x4x device
             *************************************************************************************/
    static uint8_t SPIReadChain(AS5x4x *device, uint16_t numchain, uint8_t *rddata, uint16_t size);

    void constructNOP(void);            // Function to request a NOP transmission
    void constructCEF(void);            // Function to request a CEF transmission
    void constructAGC(void);            // Function to request a AGC transmission
    void constructMag(void);            // Function to request a Mag transmission
    void constructAng(void);            // Function to request a Ang transmission

    void constructALL(void);            // Function to request full update of device

    uint8_t checkDataRequest(void);
    void readDataPacket(void);                      // Read the contents of the 16bit buffer

    static Daisy constructDaisy(AS5x4x *device, uint16_t numchain);
        // Construct the Daisy chain structure, with input parameters

    static uint8_t checkDaisyRequest(Daisy *chain);
        // Check to see if there is any new data to be transmitted for devices in the Daisy chain
    static void readDaisyPackets(Daisy *chain); // Read the contents of all devices in chain

public:     /**************************************************************************************
             * ==  PUBLIC   == >>>    POLING FUNCTIONS FOR DATA TRANSFER     <<<
             *   -----------
             *  Visible functions used to setup and transfer packets of data to the selected
             *  AS5x4x device - Will wait for any hardware registers to be in correct state before
             *  progressing.
             *************************************************************************************/
    void        poleSPITransmit(SPIPeriph *Interface, GPIO *CS);
    static void poleSPITransmit(Daisy *chain, SPIPeriph *Interface, GPIO *CS);

public:     /**************************************************************************************
             * ==  PUBLIC   == >>>   INTERRUPT FUNCTIONS FOR DATA TRANSFER   <<<
             *
             *  Visible functions used to support communication to AD741x device, on a interrupt
             *  bases.
             *      Enabling of interrupt bits/DMAs are not handled within this class.
             *************************************************************************************/
    void reInitialise(void);                                // Initialise the write/read internal
                                                            // request buffers
    void intSingleTransmit(SPIPeriph *hal_SPI, GPIO *CS, uint8_t *rBuff, uint8_t *wBuff,
                           volatile SPIPeriph::DevFlt *fltReturn, volatile uint16_t *cmpFlag,
                           uint16_t *cmpTarget);

    static void intSingleTransmit(SPIPeriph *hal_SPI, GPIO *CS, Daisy *chain,
                                  uint8_t *rBuff, uint8_t *wBuff);

    virtual ~AS5x4x();
};

#endif /* AS5X4X_H_ */
