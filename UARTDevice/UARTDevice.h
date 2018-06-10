/**************************************************************************************************
 * @file        UARTDevice.h
 * @author      Thomas
 * @version     V0.1
 * @date        10 Jun 2018
 * @brief       << Manually Entered >>
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
 *          For RaspberryPi - TBD
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
 *          ".ReceiveITEnable"      - To enable the receive interrupt trigger (without this
 *                                    interrupt will not trigger function calls).
 *          ".SingleTransmit_IT"    - Put data onto "Transmit" buffer to be set via UART (will
 *                                    enable the transmit buffer empty flag)
 *          ".SingleRead_IT"        - Pulls any new data from the "Receive" buffer
 *          ".IRQHandle"            - Interrupt handler, to be called in Interrupt Routine
 *                                      -> How to do this is detailed in the source file for this
 *                                         function
 *          ".UpdateBufferSize"     - Used to increase size of both Receive and Transmit buffers
 *
 *      There is no other functionality within this class.
 *************************************************************************************************/
#ifndef UARTDevice_H_
#define UARTDevice_H_

#include <stdint.h>
#include "GenBuffer/GenBuffer.h"        // Provide the template for the circular buffer class

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
#include "stm32f1xx_hal.h"              // Include the HAL library

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
// Add includes specific to the Raspberry Pi

#else
//==================================================================================================
#error "Unrecognised target device"

#endif

// Defines specific within this class
// None

// Types used within this class
typedef enum {
    UART_NoFault = 0,
    UART_DataError = 1,

    UART_Initialised = -1
} _UARTDevFlt;

class UARTDevice {
    // Declarations which are generic, and will be used in ALL devices
    _UARTDevFlt    Flt;                 // Fault state of the class
    GenBuffer<uint8_t>  *Receive;       // Pointer to GenBuffer class used for data "Receive"d via
                                        // UART
    GenBuffer<uint8_t>  *Transmit;      // Pointer to GenBuffer class used for data to be
                                        // "Transmit"ted via UART

// Device specific entries
#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    private:
        UART_HandleTypeDef  *UART_Handle;       // Store the UART handle

    public:
        UARTDevice(UART_HandleTypeDef *UART_Handle);    // Setup the UART class, for STM32Fxx by
                                                        // providing the UART type define handle
        UARTDevice(UART_HandleTypeDef *UART_Handle, uint32_t Buffersize);
        // Setup the UART class, for STM32Fxx by providing the UART type define handle, as well as
        // a defined value for the depth of the UART buffers


#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    private:


    public:
        UART();;

#else
//==================================================================================================
    public:
        UART();;

#endif

public:
    // 1> Following functions will WAIT for actions to complete before finishing
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    uint8_t PoleSingleRead(void);           // Will wait until there is new data to be read
    void PoleSingleTransmit(uint8_t data);  // Will wait until it can transfer data via UART
    _UARTDevFlt PoleTransmit(uint8_t *pData, uint16_t size);// Will wait until it can transfer data
                                                            // via UART, multiple entries

    // 2> Interrupt functions
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    void UpdateBufferSize(uint32_t newsize);                // Update the size of buffer

    void SingleTransmit_IT(uint8_t data);       // Add data to be transmitted via UART
    _GenBufState SingleRead_IT(uint8_t *pData); // Take data from received buffer if data is
                                                // available
    void ReceiveITEnable(void);                 // Enable the Receive Interrupt
    void IRQHandle(void);                       // Interrupt handler

    virtual ~UARTDevice();
};

#endif /* UART_UART_H_ */
