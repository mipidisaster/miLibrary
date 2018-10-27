/**************************************************************************************************
 * @file        I2CDevice.h
 * @author      Thomas
 * @version     V0.1
 * @date        27 Oct 2018
 * @brief       << Manually Entered >>
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * 
 * Class supports multiple target devices, depending upon which one is defined will modify how the
 * I2CDevice class can be initialised.
 * The basic use of the class is the same for all target devices
 *      Call Class I2CDevice to initialise the class
 *          For STM32L devices, providing the address of the I2C handler - from cubeMX
 *
 *      Depending upon how the programmer wants to use the I2C device, will change which functions
 *      are utilised.
 *      For functions to wait for new data, or data to be transmitted use (polling mode):
 *          ".poleMasterTransmit"   - Transmit data (in MASTER mode), waiting for states to be
 *                                    ready
 *          ".poleMasterReceive"    - Receive data (in MASTER mode), waiting for states to be ready
 *
 *      If interrupts are to be used, then will first need to be configured within the NVIC (which
 *      is not part of this class), then the following can be used:
 *          >> NOT SUPPORTED YET <<
 *
 *      Following functions are protected so will only work for classes which inherit from this
 *      one, and not visible external to class:
 *          ".DRRead"               - Will take data straight from hardware
 *          ".DRWrite"              - Will put data straight onto the hardware
 *          ".RequestTransfer"      - Configures the hardware for a I2C transfer (sets up Address,
 *                                    R/~W, START/STOP, etc.
 *
 *          ".TransmitEmptyChk"     - Check to see if the Transmit Empty buffer is empty
 *          ".TransmitComptChk"     - Check to see if Transmission is complete
 *          ".ReceiveToReadChk"     - Check to see if the Receive buffer is full (data to read)
 *          ".BusNACKChk"           - Check to see if there has been a NACK on the BUS
 *          ".BusBusyChk"           - Check to see if the BUS is busy
 *          ".BusSTOPChk"           - Check to see if there has been a STOP on the BUS
 *          ".BusErroChk"           - Check to see if there has been an ERROR on the BUS
 *
 *          ".ClearNACK"            - Clear the NACK status bit
 *          ".ClearBusEr"           - Clear the Bus error status bis
 *          ".Clear_STOP"           - Clear the STOP status bit
 * 
 *************************************************************************************************/
#ifndef I2CDEVICE_H_
#define I2CDEVICE_H_

#include "FileIndex.h"

#include FilInd_GENBUF_HD               // Provide the template for the circular buffer class

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
// Add includes specific to the STM32Fxx devices

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================
#include "stm32l4xx_hal.h"              // Include the HAL library

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
// Add includes specific to the Raspberry Pi

#else
//==================================================================================================
#error "Unrecognised target device"

#endif

// Defines specific within this class
//	--- Include any #defines within class - use similar naming as class
//	--- keep this relatively light, large quantities should be within a seperate header file
//	--- named <class_name>_defs.h

// Types used within this class
typedef enum {
    I2C_NoFault         = 0,
    I2C_NACK            = 1,
    I2C_BUS_ERROR       = 2,

    I2C_Initialised     = -1
}   _I2CDevFlt;

typedef enum {
    I2C_AutoEnd         = 0,        // In AutoEnd mode, internal class functions will generate STOP
                                    // command, once final packet has transmitted
    I2C_Reload          = 1,        // In Reload mode, internal class will wait for more data to be
                                    // added to the buffer before continuing transmission
    I2C_SoftEnd         = 2         // In SoftEnd mode, generation of STOP/RELOAD condition need
                                    // to be done manually
}   _I2CCommMode;

typedef enum {
    I2C_Nothing         = 0,        // Do not change the state of the current communication request
    I2C_START_WRITE     = 1,        // Start new communication, in WRITE MODE
    I2C_START_READ      = 2,        // Start new communication, in READ MODE
    I2C_STOP            = 3         // STOP current communication
}   _I2CRequest;


class I2CDevice {
    // Declarations which are generic, and will be used in ALL devices
    protected:
        uint8_t         curSize;        // Current communication packet size
        _I2CRequest     curReqst;       // Current request for communication (ignores "Nothing")

// Device specific entries
#if ( defined(zz__MiSTM32Fx__zz) || defined(zz__MiSTM32Lx__zz)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//==================================================================================================
    protected:
        I2C_HandleTypeDef  *I2C_Handle;     // Store the I2C handle

    public:
        I2CDevice(I2C_HandleTypeDef *I2C_Handle);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    public:
        I2CDevice();

#else
//==================================================================================================
    public:
        I2CDevice();

#endif
public:
    _I2CDevFlt      Flt;            // Fault state of the I2C Device

    /**********************************************************************************************
     * ## PROTECTED ## >>>     DIRECT HARDWARE READING FUNCTIONS     <<<
     *
     *  Functions allow direct access to the hardware/base functions of this class. Are not
     *  visible unless "friend" or "child"/inherited
     *********************************************************************************************/
protected:
    uint8_t DRRead(void);                   // Function to read direct from the hardware
    void DRWrite(uint8_t data);             // Function to write direct to the hardware

    uint8_t TransmitEmptyChk(void);         // Check state of transmission register (1 = empty)
    uint8_t TransmitComptChk(void);         // Check state of transmission complete (1 = cmplt)
    uint8_t ReceiveToReadChk(void);         // Check state of receive data register (1 =  read)

    uint8_t BusNACKChk(void);               // Check if No Acknowledge is received  (1 = NACK)
    void ClearNACK(void);                   // Clear the NACK bit
    uint8_t BusBusyChk(void);               // Check if I2C bus is busy             (1 = BUSY)
    uint8_t BusSTOPChk(void);               // Check if I2C bus STOP has been set   (1 = STOP)
    uint8_t BusErroChk(void);               // Check if START/STOP bit is set wrong (1 =  Flt)
    void ClearBusEr(void);                  // Clear the Bus error flag
    void Clear_STOP(void);                  // Clear the Bus STOP bit

    void RequestTransfer(uint16_t devAddress, uint8_t size, _I2CCommMode mode, _I2CRequest reqst);
        // Request a new communicate via I2C device, function handles the START/STOP, R/~W setup

    /**********************************************************************************************
     * ##  PUBLIC   ## >>>    POLING FUNCTIONS FOR DATA TRANSFER     <<<
     *
     *  Visible functions used to transfer data via I2C - will wait for any registers to be in
     *  correct state before progressing.
     *********************************************************************************************/
public:
    _I2CDevFlt poleMasterTransmit(uint16_t devAddress, uint8_t *pdata, uint8_t size);
    _I2CDevFlt poleMasterReceive(uint16_t devAddress, uint8_t *pdata, uint8_t size);

    /**********************************************************************************************
     * ##  PUBLIC   ## >>>    INTERRUPT FUNCTIONS FOR DATA TRANSFER    <<<
     *
     *  Visible functions used to transfer data via I2C - in Interrupt mode, so allows other
     *  functions to run, whilst hardware "does its thing!"
     *********************************************************************************************/
public:

    /**********************************************************************************************
     * ##  PUBLIC   ## >>>    DMA FUNCTIONS FOR DATA TRANSFER    <<<
     *
     *  Visible functions used to transfer data via DMA I2C - all handling of data to the I2C
     *  hardware is managed outside of processor, and managed by dedicated hardware.
     *********************************************************************************************/
public:


    virtual ~I2CDevice();
};

#endif /* I2CDEVICE_H_ */
