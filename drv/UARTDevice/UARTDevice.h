/**************************************************************************************************
 * @file        UARTDevice.h
 * @author      Thomas
 * @version     V1.1
 * @date        06 Oct 2018
 * @brief       Header file for the Generic UART Class handle
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * Class supports multiple target devices, depending upon which one is defined will modify how the
 * UART class can be initialised.
 * The basic use of the class is the same for all target devices
 *      Call Class UARTDevice to initialise the class
 *          For STM32F devices, providing the address of the UART handler - from cubeMX
 *          For STM32L devices, providing the address of the UART handler - from cubeMX
 *          For RaspberryPi, provide the location of the serial interface, and the desired baudrate
 *
 *          Additional to this the size of the UART buffer array is required.
 *
 *      Depending upon how the programmer wants to use the UART device, will change which functions
 *      are utilised.
 *      For functions to wait for new data, or data to be transmitted use:
 *          ".PoleSingleRead"       - Wait for new data to be read via UART
 *          ".PoleSingleTransmit"   - Wait for data to be transmitted via UART
 *          ".PoleTransmit"         - As above, however can transmit multiple data (array)
 *
 *      If interrupts are to be used, then will first need to be configured within the NVIC (which
 *      is not part of this class), then the following can be used
 *          ".ReceiveIT(<intr>)"    - To enable/disable Read data register is empty
 *          ".TransmtIT(<intr>)"    - To enable/disable Transmit data register is empty
 *          ".TransCmIT(<intr>)"    - To enable/disable Transmission Complete register interrupt
 *
 *          ".SingleTransmit_IT"    - Put data onto "Transmit" buffer to be set via UART (will
 *                                    enable the transmit buffer empty flag)
 *          ".SingleRead_IT"        - Pulls any new data from the "Receive" buffer
 *
 *          ".IRQHandle"            - Interrupt handler, to be called in Interrupt Routine
 *                                      -> How to do this is detailed in the source file for this
 *                                         function
 *
 *      Following functions are protected, so will only work for classes which inherit from this
 *      one, and not visible external to class:
 *          ".DRRead"               - Will take data straight from the hardware
 *          ".DRWrite"              - Will put data straight onto the hardware
 *          ".TransmitEmptyITCheck" - Check state of the Transmit hardware interrupt, if data can
 *                                    be added to hardware queue output will be 0x01, otherwise
 *                                    0x00.
 *          ".ReceiveDataToReadChk" - Check state of the Receive hardware interrupt, if data is
 *                                    ready to be read from hardware output will be 0x01, otherwise
 *                                    0x00.
 *          ".TransmitComptITCheck" - Check state of the Transmission Complete hardware interrupt,
 *                                    if data has been transmitted then 0x01, otherwise 0x00.
 *
 *  If "__LiteImplement__" has been defined, then the class will not use "use" or "delete" to
 *  minimise the size impact. Therefore fully defined "GenBuffers" need to be provided to the
 *  constructors of the UARTDevice class.
 *
 *      There is no other functionality within this class.
 *************************************************************************************************/
#ifndef UARTDevice_H_
#define UARTDevice_H_

#include "FileIndex.h"
#include <stdint.h>

#include FilInd_GENBUF_HD               // Provide the template for the circular buffer class

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
#include "stm32f1xx_hal.h"              // Include the HAL library

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================
#include "stm32l4xx_hal.h"              // Include the HAL library

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
#include <wiringSerial.h>               // Include the wiringPi UART/Serial library

#else
//==================================================================================================
#error "Unrecognised target device"

#endif

// Defines specific within this class
#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
// None

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================
// None

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
#define UARTD_ReceiveIntBit     0       // Define the bit position for enabling Receive interrupt
#define UARTD_TransmtIntBit     1       // Define the bit position for enabling Transmit interrupt
#define UARTD_TransCmIntBit     2       // Define the bit position for enabling Transmit Complete

#define UARTD_EnabInter(reg, bit)  ((reg) |=  (0x01 << bit))    // Enable specified bit
#define UARTD_DisaInter(reg, bit)  ((reg) &= ~(0x01 << bit))    // Disable specified bit

#else
//==================================================================================================
// None

#endif

// Types used within this class
typedef enum {
    UART_NoFault        = 0,
    UART_DataError      = 1,
    UART_Parity         = 2,

    UART_Initialised    = -1
} _UARTDevFlt;

typedef enum {
    UART_Enable = 0,
    UART_Disable = 1
} _UARTITState;

class UARTDevice {
    // Declarations which are generic, and will be used in ALL devices
    protected:
        _UARTDevFlt    Flt;                 // Fault state of the class
        GenBuffer<uint8_t>  *Receive;       // Pointer to GenBuffer class used for data "Receive"d
                                            // via UART
        GenBuffer<uint8_t>  *Transmit;      // Pointer to GenBuffer class used for data to be
                                            // "Transmit"ted via UART

// Device specific entries
#if ( defined(zz__MiSTM32Fx__zz) || defined(zz__MiSTM32Lx__zz)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//==================================================================================================
    protected:
        UART_HandleTypeDef  *UART_Handle;       // Store the UART handle

    public:
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#ifdef __LiteImplement__        // If "__LiteImplement__" has been defined, then need to have array
                                // fully defined, and provided to the "GenBuffer"
                                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    public:
        UARTDevice(UART_HandleTypeDef *UART_Handle,
                   GenBuffer<uint8_t> *receivearray, GenBuffer<uint8_t> *transmitarray);
        // Setup the UART class, for STM32Fxx by providing the UART type define handle, as well the
        // "GenBuffer" needing to be provided to the function, to be fully defined outside of class

#else                           // If "__LiteImplement__" has not been defined, then allow use of
                                // "new" and "delete" for defining internal arrays
                                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    public:
        UARTDevice(UART_HandleTypeDef *UART_Handle, uint32_t Buffersize);
        // Setup the UART class, for STM32Fxx by providing the UART type define handle, as well as
        // a defined value for the depth of the UART buffers

#endif                          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    private:
        int                 UART_Handle;        // Stores the device to communicate too
        const char          *deviceloc;         // Store location file for UART device
        int                 baudrate;           // Store entered baudrate
        uint8_t             pseudo_interrupt;   // Pseudo interrupt register

    public:
        int  AnySerDataAvil(void);              // Function to provide the amount of data at
                                                // hardware baundry

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#ifdef __LiteImplement__        // If "__LiteImplement__" has been defined, then need to have array
                                // fully defined, and provided to the "GenBuffer"
                                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    public:
        UARTDevice(const char *deviceloc, int baud,
                   GenBuffer<uint8_t> *receivearray, GenBuffer<uint8_t> *transmitarray);
        // Setup the UART class, by providing the folder location of serial interface, and baudrate
        // as well the "GenBuffer" needing to be provided to the function, to be fully defined
        // outside of class

#else                           // If "__LiteImplement__" has not been defined, then allow use of
                                // "new" and "delete" for defining internal arrays
                                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    public:
        UARTDevice(const char *deviceloc, int baud, uint32_t Buffersize);
        // Setup the UART class, by providing the folder location of serial interface, and baudrate
        // and define the required Buffersize

#endif                          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#else
//==================================================================================================
    public:
        UARTDevice();

#endif

protected:
    //0> Hardware reading
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    uint8_t DRRead(void);                       // Function to read direct from the hardware
    void DRWrite(uint8_t data);                 // Function to write direct to the hardware

    uint8_t TransmitEmptyITCheck(void);         // Check state of transmission hardware buffer
    uint8_t TransmitComptITCheck(void);         // Check state of transmission complete hardware
    uint8_t ReceiveDataToReadChk(void);         // Check state of receive data from hardware

public:
    // 1> Following functions will WAIT for actions to complete before finishing
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    uint8_t PoleSingleRead(void);           // Will wait until there is new data to be read
    void PoleSingleTransmit(uint8_t data);  // Will wait until it can transfer data via UART
    _UARTDevFlt PoleTransmit(uint8_t *pData, uint16_t size);// Will wait until it can transfer data
                                                            // via UART, multiple entries

    // 2> Interrupt functions
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    void ReceiveIT(_UARTITState intr);          // Controller for enabling/disabling Receive IT
    void TransmtIT(_UARTITState intr);          // Controller for enabling/disabling Transmit IT
    void TransCmIT(_UARTITState intr);          // Controller for enabling/disabling Transmit
                                                // Complete IT

    void SingleTransmit_IT(uint8_t data);       // Add data to be transmitted via UART
    _GenBufState SingleRead_IT(uint8_t *pData); // Take data from received buffer if data is
                                                // available

    virtual void IRQHandle(void);               // Interrupt handler

    virtual ~UARTDevice();
};

#endif /* UART_UART_H_ */
