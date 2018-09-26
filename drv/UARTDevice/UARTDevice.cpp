/**************************************************************************************************
 * @file        UART.cpp
 * @author      Thomas
 * @version     V0.6
 * @date        11 Jul 2018
 * @brief       Source file for the Generic UART Class handle
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include <UARTDevice/UARTDevice.h>

uint8_t UARTDevice::DRRead(void) {
/**************************************************************************************************
 * Read from the UART hardware
 *************************************************************************************************/

    #if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    return ((uint8_t)this->UART_Handle->Instance->DR);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    return((uint8_t) serialGetchar(this->UART_Handle));

#else
//==================================================================================================

#endif
}

void UARTDevice::DRWrite(uint8_t data) {
/**************************************************************************************************
 * Write from the UART hardware
 *************************************************************************************************/

    #if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    this->UART_Handle->Instance->DR = data;

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    serialPutchar(this->UART_Handle, (unsigned char) data);

#else
//==================================================================================================

#endif
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#ifdef __LiteImplement__        // If "__LiteImplement__" has been defined, then need to have array
                                // fully defined, and provided to the "GenBuffer"
                                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
UARTDevice::UARTDevice(UART_HandleTypeDef *UART_Handle,
                       GenBuffer<uint8_t> *receivearray, GenBuffer<uint8_t> *transmitarray) {
/**************************************************************************************************
 * Create a UART class specific for the STM32F device. Also need to provide the fully defined
 * "GenBuffer" class for both "Receive" and "Transmit" buffers.
 *
 * As the STM32CubeMX already pre-generates the setting up and configuring of the desired UART
 * device, there is no need to define that within this function. Simply providing the handle is
 * required.
 *************************************************************************************************/
    this->UART_Handle   = UART_Handle;      // Copy data into class
    this->Flt           = UART_Initialised; // Initialise the fault to "initialised"

    // Configure both the Input and Output Buffers to be the size as per input
    this->Receive       = receivearray;     // Link the internal "Receive" buffer pointer to
                                            // provided Receive Buffer
    this->Transmit      = transmitarray;    // Link the internal "Transmit" buffer pointer to
                                            // provided Transmit Buffer
}

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
UARTDevice::UARTDevice(const char *deviceloc, int baud,
                       GenBuffer<uint8_t> *receivearray, GenBuffer<uint8_t> *transmitarray)) {
/**************************************************************************************************
 * Create a UART class specific for the Raspberry Pi
 *  As the fully defined "GenBuffer" classes for both "Receive" and "Transmit" buffers have been
 *  provided, this is to be linked to internals of the class.
 *
 *  This will then open up the serial interface, and configure a "pseudo_interrutp" register, so
 *  as to provide the Raspberry Pi the same function use as other embedded devices.
 *  The Receive and Transmit buffers size will be as per input "BufferSize"
 *************************************************************************************************/
    this->deviceloc         = deviceloc;    // Capture the folder location of UART device
    this->baudrate          = baud;         // Capture the desired baud rate
    this->pseudo_interrupt  = 0x00;         // pseudo interrupt register used to control the UART
                                            // interrupt for Raspberry Pi

    this->Flt           = UART_Initialised; // Initialise the fault to "initialised"

    // Configure both the Input and Output Buffers to be the size as per input
    this->Receive       = receivearray;     // Link the internal "Receive" buffer pointer to
                                            // provided Receive Buffer
    this->Transmit      = transmitarray;    // Link the internal "Transmit" buffer pointer to
                                            // provided Transmit Buffer

    this->UART_Handle = serialOpen(this->deviceloc, this->baudrate);
            // Open the serial interface
}

int  UARTDevice::AnySerDataAvil(void) {
/**************************************************************************************************
* RaspberryPi specific function to determine amount of data within the hardware
*************************************************************************************************/
   return(serialDataAvail(this->UART_Handle));
}
#else
//==================================================================================================
UARTDevice::UARTDevice() {

}
#endif

#else                           // If "__LiteImplement__" has not been defined, then allow use of
                                // "new" and "delete" for defining internal arrays
                                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
UARTDevice::UARTDevice(UART_HandleTypeDef *UART_Handle, uint32_t Buffersize) {
/**************************************************************************************************
 * Create a UART class specific for the STM32F device. Also generate Receive/Transmit buffers are
 * per the input defined size.
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
UARTDevice::UARTDevice(const char *deviceloc, int baud, uint32_t Buffersize) {
/**************************************************************************************************
 * Create a UART class specific for the Raspberry Pi
 *  The provided information is the folder location of the serial interface in text, along with the
 *  desired baudrate of the serial interface.
 *
 *  This will then open up the serial interface, and configure a "pseudo_interrutp" register, so
 *  as to provide the Raspberry Pi the same function use as other embedded devices.
 *  The Receive and Transmit buffers size will be as per input "BufferSize"
 *************************************************************************************************/
    this->deviceloc         = deviceloc;    // Capture the folder location of UART device
    this->baudrate          = baud;         // Capture the desired baud rate
    this->pseudo_interrupt  = 0x00;         // pseudo interrupt register used to control the UART
                                            // interrupt for Raspberry Pi

    this->Flt           = UART_Initialised; // Initialise the fault to "initialised"

    // Configure both the Input and Output Buffers to be the size as per input
    this->Receive       = new GenBuffer<uint8_t>(Buffersize);
    this->Transmit      = new GenBuffer<uint8_t>(Buffersize);

    this->UART_Handle = serialOpen(this->deviceloc, this->baudrate);
            // Open the serial interface
}

int  UARTDevice::AnySerDataAvil(void) {
/**************************************************************************************************
 * RaspberryPi specific function to determine amount of data within the hardware
 *************************************************************************************************/
    return(serialDataAvail(this->UART_Handle));
}
#else
//==================================================================================================
UARTDevice::UARTDevice() {

}
#endif
#endif                          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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

    return (this->DRRead());  // Retrieve the read data, and pass out of function

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    int readbackdata = -1;      // Create a variable which will contain the read contents of the
                                // UART device
                                // Set this to -1, as this indicates that there is no data to be
                                // read, and need to loop until data is read
    while(readbackdata == -1) { // Loop until get data from UART
        readbackdata = this->DRRead();                  // Read data from UART
            // Function will time out after 10s returning -1. As want to pole until new data is
            // available, keep looping until get anything but -1
    }

    return ((uint8_t) readbackdata);    // If get to this point data has been read from UART,
                                        // therefore return read value
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

    this->DRWrite(data);    // Now put the selected data onto the Data Register (DR) for
                            // transmission.

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    this->DRWrite(data);    // Send data via UART

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

    // Wait for final data point to have completed transmission before continuing
    while (__HAL_UART_GET_FLAG(this->UART_Handle, UART_FLAG_TC) == RESET) {}

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    char *newpa;                        // Create a character pointer
    uint32_t i;                         // Loop variable to go through contents of array

    newpa = new char[size];             // Define an array to contain the data in a char type

    for (i = 0; i != size; i++) {       // Cycle through the size of the input array
        newpa[i] = *pData;              // Copy data into new array
        pData += sizeof(uint8_t);       // Increment the input array pointer
    }

    serialPuts(this->UART_Handle, newpa);   // Then send new data via UART
    delete [] newpa;                        // Delete the new array, such as to clear up resources

#else
//==================================================================================================

#endif

    return (this->Flt = UART_NoFault);  // If have made it to this point, then there is no issues
                                        // also update fault state

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
    this->TransmtIT(UART_Enable);                           // Enable the transmit interrupt

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    this->Transmit->InputWrite(data);                       // Add data to the Transmit buffer
    this->TransmtIT(UART_Enable);                           // Enable the transmit interrupt
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
    return (this->Receive->OutputRead(pData));  // Call the "OutputRead" function from the
        // GenBuffer class
#else
//==================================================================================================

#endif
}

void UARTDevice::ReceiveIT(_UARTITState intr) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable/disable the Receive interrupt event.
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    if (intr == UART_Enable)
        __HAL_UART_ENABLE_IT(this->UART_Handle, UART_IT_RXNE);  // Enable the RXNE interrupt

    else
        __HAL_UART_DISABLE_IT(this->UART_Handle, UART_IT_RXNE); // Disable the RXNE interrupt

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    if (intr == UART_Enable)
        UARTD_EnabInter(this->pseudo_interrupt, UARTD_ReceiveIntBit);
        // Enable the pseudo Receive bit - via the "Pseudo interrupt" register

    else
        UARTD_DisaInter(this->pseudo_interrupt, UARTD_ReceiveIntBit);
        // Disable the pseudo Receive bit - via the "Pseudo interrupt" register

#else
//==================================================================================================

#endif
}

void UARTDevice::TransmtIT(_UARTITState intr) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable/disable the Transmit interrupt event.
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    if (intr == UART_Enable)
        __HAL_UART_ENABLE_IT(this->UART_Handle, UART_IT_TXE);   // Enable the TXE interrupt

    else
        __HAL_UART_DISABLE_IT(this->UART_Handle, UART_IT_TXE);  // Disable the TXE interrupt

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    if (intr == UART_Enable)
        UARTD_EnabInter(this->pseudo_interrupt, UARTD_TransmtIntBit);
        // Enable the pseudo Transmit bit - via the "Pseudo interrupt" register

    else
        UARTD_DisaInter(this->pseudo_interrupt, UARTD_TransmtIntBit);
        // Disable the pseudo Transmit bit - via the "Pseudo interrupt" register

#else
//==================================================================================================

#endif
}

void UARTDevice::TransCmIT(_UARTITState intr) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable/disable the Transmit Complete interrupt event.
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    if (intr == UART_Enable)
        __HAL_UART_ENABLE_IT(this->UART_Handle, UART_IT_TC);    // Enable the TC interrupt

    else
        __HAL_UART_DISABLE_IT(this->UART_Handle, UART_IT_TC);   // Disable the TC interrupt

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    if (intr == UART_Enable)
        UARTD_EnabInter(this->pseudo_interrupt, UARTD_TransCmIntBit);
        // Enable the pseudo Transmit complete bit - via the "Pseudo interrupt" register

    else
        UARTD_DisaInter(this->pseudo_interrupt, UARTD_TransCmIntBit);
        // Disable the pseudo Transmit complete bit - via the "Pseudo interrupt" register

#else
//==================================================================================================

#endif
}

uint8_t UARTDevice::ReceiveDataToReadChk(void) {
/**************************************************************************************************
 * INTERRUPTS CHECKS:
 * Check the state of the Read data register not empty interrupt - which indicates that data is
 * ready to be read from the hardware.
 * Will return (0x01) if data can be read, and (0x00) if not.
 *************************************************************************************************/
#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    uint32_t isrflags   = READ_REG(this->UART_Handle->Instance->SR);
        // Get the Interrupt flags

    uint32_t cr1its     = READ_REG(this->UART_Handle->Instance->CR1);
        // Add status flags for Interrupts

    // If the Receive Data Interrupt has been triggered AND is enabled as Interrupt then ...
    if(((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
        return(0x01);

    else
        return(0x00);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    // Check to see if Receive Interrupt bit has been set.
    if (((this->pseudo_interrupt & (0x01<<UARTD_ReceiveIntBit)) == (0x01<<UARTD_ReceiveIntBit)))
        return (0x01);

    else
        return(0x00);

#else
//==================================================================================================

#endif
}

uint8_t UARTDevice::TransmitEmptyITCheck(void) {
/**************************************************************************************************
 * INTERRUPTS CHECKS:
 * Check the state of the Transmit data empty interrupt - which indicates that data can be added
 * onto hardware queue to be transmitted.
 * Will return (0x01) if data can be added, and (0x00) if not.
 *************************************************************************************************/
#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    uint32_t isrflags   = READ_REG(this->UART_Handle->Instance->SR);
        // Get the Interrupt flags

    uint32_t cr1its     = READ_REG(this->UART_Handle->Instance->CR1);
        // Add status flags for Interrupts

    // If the Data Empty Interrupt has been triggered AND is enabled as Interrupt then...
    if(((isrflags & USART_SR_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) != RESET))
        return(0x01);

    else
        return(0x00);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    // Check to see if Transmit Interrupt bit has been set.
    if ((this->pseudo_interrupt & (0x01<<UARTD_TransmtIntBit)) == (0x01<<UARTD_TransmtIntBit))
        return (0x01);

    else
        return(0x00);

#else
//==================================================================================================

#endif
}

uint8_t UARTDevice::TransmitComptITCheck(void) {
/**************************************************************************************************
 * INTERRUPTS CHECKS:
 * Check the state of the Transmission complete interrupt - which incidest that data has completed
 * tranmission
 * Will return (0x01) if data can be added, and (0x00) if not.
 *************************************************************************************************/
#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
    uint32_t isrflags   = READ_REG(this->UART_Handle->Instance->SR);
        // Get the Interrupt flags

    uint32_t cr1its     = READ_REG(this->UART_Handle->Instance->CR1);
        // Add status flags for Interrupts

    // If the Tranmission has completed been triggered AND is enabled as Interrupt then...
    if(((isrflags & USART_SR_TC) != RESET) && ((cr1its & USART_CR1_TCIE) != RESET)) {
        this->UART_Handle->Instance->SR &= ~(USART_SR_TC);  // Clear the status register
        return(0x01);
    }

    else
        return(0x00);

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    // Check to see if Transmit Complete Interrupt bit has been set.
    if ((this->pseudo_interrupt & (0x01<<UARTD_TransCmIntBit)) == (0x01<<UARTD_TransCmIntBit))
        return (0x01);

    else
        return(0x00);

#else
//==================================================================================================

#endif
}

void UARTDevice::IRQHandle(void) {
/**************************************************************************************************
 * INTERRUPTS:
 * Interrupt Service Routine for the UART class. Each of the supported devices needs to call this
 * function in different ways - therefore each implementation is mentioned within the coded section.
 *
 * Function will then read the hardware status flags, and determine which interrupt has been
 * triggered:
 *      If receive interrupt enabled, then data from Data Register is stored onto the "Receive"
 *      buffer
 *
 *      If transmission interrupt enabled, then data from "Transmit" buffer is put onto the Data
 *      Register for transmission.
 *
 *      No other interrupts are currently supported.
 *************************************************************************************************/

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
/**************************************************************************************************
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
 *      UART *UARTDevice;
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
    uint8_t tempdata = 0x00;    // Temporary variable to store data for UART

    if (this->TransmitComptITCheck() == 0x01) {}

    // If the Receive Data Interrupt has been triggered AND is enabled as Interrupt then ...
    if(this->ReceiveDataToReadChk() == 0x01) {
        tempdata  = this->DRRead();                     // Read data and put onto temp variable
        this->Receive->InputWrite((uint8_t) tempdata);  // Add to Receive buffer
    }

    // If the Data Empty Interrupt has been triggered AND is enabled as Interrupt then...
    if(this->TransmitEmptyITCheck() == 0x01) {
        // If there is data to be transmitted, then take from buffer and transmit
        if (this->Transmit->OutputRead(&tempdata) != GenBuffer_Empty) {
            this->DRWrite((uint8_t)tempdata);
        }
        else {      // Otherwise, disable the TXE interrupt
            this->TransmtIT(UART_Disable);
        }
    }

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
/**************************************************************************************************
 * Example of call.
 *  As setting up a real interrupt for Raspberry Pi involves hacking the kernel, which I am not
 *  doing, the route I have taken is to use threads - pthreads.
 *  What this involves is creating a separate stream (thread) which will just check the size of the
 *  UART peripheral buffer or if data has been requested to be sent (via pseudo interrupt
 *  register). Then the main thread can use the Read/Transmit functions as normal.
 *
 *  So setting up the pthread:
 *  The -pthread needs to be added to the G++ linker library, won't require the "-" within eclipse,
 *  however for "CMakeList", will need to be written as "-lpthread".
 *  Then within the main code add "include <pthread.h>
 *
 *  This will then allow you to create a new stream:
 *  main.c {
 *  pthread_t thread;   // Create thread handle
 *  if (pthread_create(&thread, NULL, &threadfunction, NULL) != 0) {
 *      std::cout << "Failed to create thread" << std::endl;
 *  }
 *
 * pthread_create will return 0 if it has created a new thread. In the example above "&thread" is
 * the thread handle created above, next entry ...no idea... 3rd entry is a pointer to the desired
 * function call, 4th can be used to provide values to the function - however I haven't tried this.
 *
 * void *threadfunction(void *value) {
 *  uint8_t UARTReturn;
 *  while (1) {
 *      delay(100);
 *      MyUART->IRQHandle();
 *  }
 *
 *  return 0;
 * }
 *
 * Similar to the STM32 a pointer to the UARTDevice will need to be made global to allow this
 * new thread to all the "IRQHandler"
 *************************************************************************************************/
    int BufferContents = 0;             // Variable to store the amount of data in UART peripheral

    if (this->ReceiveDataToReadChk() == 0x01) {}

    // Check to see if Receive Interrupt bit has been set.
    if (this->ReceiveDataToReadChk() == 0x01) {
        // If it has check to see if there is any data to be read
        BufferContents = serialDataAvail(this->UART_Handle);    // Get the amount of data in UART

        while (BufferContents > 0) {
            this->Receive->InputWrite(this->DRRead());
            BufferContents--;
        }
    }

    uint8_t tempdata;

    if(this->TransmitEmptyITCheck() == 0x01) {
        while (this->Transmit->OutputRead(&tempdata) != GenBuffer_Empty) {
            this->DRWrite(tempdata);
        }

        this->TransmtIT(UART_Disable);
    }

#else
//==================================================================================================

#endif
}

UARTDevice::~UARTDevice() {
/**************************************************************************************************
 * When the destructor is called, need to ensure that the memory allocation is cleaned up, so as
 * to avoid "memory leakage"
 *************************************************************************************************/
#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    serialClose(this->UART_Handle);     // Close the UART interface

#else
//==================================================================================================

#endif
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#ifdef __LiteImplement__        // If "__LiteImplement__" has been defined, then need to have array
                                // fully defined, and provided to the "GenBuffer"
                                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#else                           // If "__LiteImplement__" has not been defined, then allow use of
                                // "new" and "delete" for defining internal arrays
                                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    delete [] Receive;              // Delete the array "In"
    delete [] Transmit;             // Delete the array "Out"

#endif                          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

