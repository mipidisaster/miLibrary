/**************************************************************************************************
 * @file        UART.cpp
 * @author      Thomas
 * @version     V0.1
 * @date        10 Jun 2018
 * @brief       << Manually Entered >>
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include <UARTDevice/UARTDevice.h>

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
UARTDevice::UARTDevice(UART_HandleTypeDef *UART_Handle) {
/**************************************************************************************************
 * Create a UART class specific for the STM32F device
 *  Generate a default size for the Receive and Transmit buffers.
 *
 * As the STM32CubeMX already pre-generates the setting up and configuring of the desired UART
 * device, there is no need to define that within this function. Simply providing the handle is
 * required.
 *************************************************************************************************/
    this->UART_Handle   = UART_Handle;  // Copy data into class
    this->Flt           = UART_Initialised; // Initialise the fault to "initialised"

    this->Receive       = new GenBuffer<uint8_t>(128);  // Configure Receive Buffer to 128 deep
    this->Transmit      = new GenBuffer<uint8_t>(128);  // Configure Transmit Buffer to 128 deep
}

UARTDevice::UARTDevice(UART_HandleTypeDef *UART_Handle, uint32_t Buffersize) {
/**************************************************************************************************
 * Create a UART class specific for the STM32F device. Also
 *  Generate Receive/Transmit buffers are per the input defined size.
 *
 * As the STM32CubeMX already pre-generates the setting up and configuring of the desired UART
 * device, there is no need to define that within this function. Simply providing the handle is
 * required.
 *************************************************************************************************/
    this->UART_Handle   = UART_Handle;  // Copy data into class
    this->Flt           = UART_Initialised; // Initialise the fault to "initialised"

    // Configure both the Input and Output Buffers to be the size as per input
    this->Receive       = new GenBuffer<uint8_t>(Buffersize);
    this->Transmit      = new GenBuffer<uint8_t>(Buffersize);
}

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
UARTDevice::UARTDevice() {

}
#else
//==================================================================================================
UARTDevice::UARTDevice() {

}
#endif

uint8_t UARTDevice::PoleSingleRead(void) {
/**************************************************************************************************
 * Will take a single byte of data from the UART peripheral, and return out of function.
 * Note that this will WAIT until there is data available to be read.
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    // Ensure that the UART interface has been enabled
    if ((this->UART_Handle->Instance->CR1 & USART_CR1_UE) != USART_CR1_UE)
        __HAL_UART_ENABLE(this->UART_Handle);   // Enable the UART interface if not enabled

        // Check to see if there is data to be read, done by checking the Read Data Register not
        // empty flag (RXNE), if this is TRUE then there is data to be read.
    while (__HAL_UART_GET_FLAG(this->UART_Handle, UART_FLAG_RXNE) != SET) {}

    return ((uint8_t)this->UART_Handle->Instance->DR);  // Retrieve the read data, and pass out
                                                        // of function

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================

#else
//==================================================================================================
    return(0);
#endif
}

void UARTDevice::PoleSingleTransmit(uint8_t data) {
/**************************************************************************************************
 * Will take the provided data, and put onto the UART peripheral for transmission.
 * Note that this will WAIT until it is able to transmit the data.
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    // Ensure that the UART interface has been enabled
    if ((this->UART_Handle->Instance->CR1 & USART_CR1_UE) != USART_CR1_UE)
        __HAL_UART_ENABLE(this->UART_Handle);   // Enable the UART interface if not enabled

        // Check to see if the Transmit Data Register is empty (TXE), this will be set to TRUE
        // by the hardware to indicate that the contents of the TDR register have been setup for
        // transmission. Therefore new data can be added for transmission
    while (__HAL_UART_GET_FLAG(this->UART_Handle, UART_FLAG_TXE) == RESET) {}

    this->UART_Handle->Instance->DR = data;     // Now put the selected data onto the Data Register
                                                // (DR) for transmission.

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================

#else
//==================================================================================================

#endif
}

_UARTDevFlt UARTDevice::PoleTransmit(uint8_t *pData, uint16_t size) {
/**************************************************************************************************
 * An extension of the Single Transmit function, this allows for an array of data to be set via
 * UART.
 * Again it will wait until it can transmit, and all data has been transmitted before exiting.
 * > However if there is no data, or the size is zero, it will return a fault.
 *************************************************************************************************/

    if (pData == __null || size == 0)       // If no data has been requested to be set
        return (this->Flt = UART_DataError);// return error with DATA (also update fault state)

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    while (size > 0) {                      // Whilst there is data to be transferred
        this->PoleSingleTransmit(*pData);   // Transmit the single point of data
        pData += sizeof(uint8_t);           // Increment pointer by the size of the data to be
                                            // transmitted
        size--;                             // Decrement the size
    }

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================

#else
//==================================================================================================

#endif

    return (this->Flt = UART_NoFault);  // If have made it to this point, then there is no issues
                                        // also update fault state

}

void UARTDevice::UpdateBufferSize(uint32_t newsize) {
/**************************************************************************************************
 * Function will increase the size of both the Receive and Transmit buffers, to the new input size
 * -> this will retain the current data in buffer and pointers, if they are still within the
 *    bounds of the new size.
 *************************************************************************************************/
    this->Receive->SizeUpdate(newsize);     // Update the Receive buffer to new defined size
    this->Transmit->SizeUpdate(newsize);    // Update the Transmit buffer to new defined size
}


void UARTDevice::SingleTransmit_IT(uint8_t data) {
/**************************************************************************************************
 * INTERRUPTS:
 * The will add the input data onto the Transmit buffer, for it to be handled by the UART
 * interrupt. - The UART class version of the interrupt handler is ".IRQHandle"
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    this->Transmit->InputWrite(data);                       // Add data to the Transmit buffer
    __HAL_UART_ENABLE_IT(this->UART_Handle, UART_IT_TXE);   // Enable the Transmit Data interrupt

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================

#else
//==================================================================================================

#endif
}

_GenBufState UARTDevice::SingleRead_IT(uint8_t *pData) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will read the latest read packet via UART, which will be stored within the "Received"
 * buffer. If there is no new data, it will return the "GenBuffer_Empty" value.
 * The UART class version of the interrupt handler is ".IRQHandle"
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    return (this->Receive->OutputRead(pData));  // Call the "OutputRead" function from the
        // GenBuffer class

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================

#else
//==================================================================================================

#endif
}

void UARTDevice::ReceiveITEnable(void) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable the Receive interrupt event.
 *************************************************************************************************/
#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    __HAL_UART_ENABLE_IT(this->UART_Handle, UART_IT_RXNE);  // Enable the RXNE interrupt

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================

#else
//==================================================================================================

#endif
}

void UARTDevice::IRQHandle(void) {
/**************************************************************************************************
 * INTERRUPTS:
 * Interrupt Service Routine for the UART class. This needs to be called within the desired
 * peripheral interrupt function all - an example is shown below.
 * Function will then read the hardware status flags, and determine which interrupt has been
 * triggered:
 *      If receive interrupt enabled, then data from Data Register is stored onto the "Receive"
 *      buffer
 *
 *      If transmission interrupt enabled, then data from "Transmit" buffer is put onto the Data
 *      Register for transmission.
 *
 *      No other interrupts are currently supported.
 *
 * Example of call.
 *  As the main.h/main.c are included within the interrupt header and source file, the function
 *  call needs to be setup there.
 *  A global pointer to the UART class needs to be setup, then within the main() routine, it needs
 *  to be configured to the desired settings (STM32CubeMX - handle linked to class).
 *
 *  Then external function needs to call ".IRQHandle", and then be called within the interrupt
 *  function call:
 *
 *  main.c
 *      * Global pointer
 *      static UART *UARTDevice;
 *
 *      main() {
 *      UARTDevice = new UART(&huart1);
 *      while() {};
 *      }
 *
 *      void UARTDevice_IRQHandler(void) {  UARTDevice->IRQHandle();  }
 *
 *  main.h
 *      extern "C" {
 *      void UARTDevice_IRQHandler(void);
 *      }
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    uint32_t isrflags   = READ_REG(this->UART_Handle->Instance->SR);
        // Get the Interrupt flags

    uint32_t cr1its     = READ_REG(this->UART_Handle->Instance->CR1);
        // Add status flags for Interrupts

    uint8_t tempdata = 0x00;    // Temporary variable to store data for UART

    // If the Receive Data Interrupt has been triggered AND is enabled as Interrupt then ...
    if(((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET)) {
        tempdata  = this->UART_Handle->Instance->DR;    // Read data and put onto temp variable
        this->Receive->InputWrite((uint8_t) tempdata);  // Add to Receive buffer
    }

    // If the Data Empty Interrupt has been triggered AND is enabled as Interrutp then...
    if(((isrflags & USART_SR_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) != RESET)) {
        // If there is data to be transmitted, then take from buffer and transmit
        if (this->Transmit->OutputRead(&tempdata) != GenBuffer_Empty) {
            this->UART_Handle->Instance->DR = (uint8_t)tempdata;
        }
        else {      // Otherwise, disable the TXE interrupt
            __HAL_UART_DISABLE_IT(this->UART_Handle, UART_IT_TXE);
        }
    }

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================

#else
//==================================================================================================

#endif
}

UARTDevice::~UARTDevice() {
/**************************************************************************************************
 * When the destructor is called, need to ensure that the memory allocation is cleaned up, so as
 * to avoid "memory leakage"
 *************************************************************************************************/
    delete [] Receive;              // Delete the array "In"
    delete [] Transmit;             // Delete the array "Out"
}

