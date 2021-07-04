/**************************************************************************************************
 * @file        UARTDMAPeriph.cpp
 * @author      Thomas
 * @version     V0.2
 * @date        28 Sep 2019
 * @brief       Source file for the Generic UART with DMA Class handle
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include <FileIndex.h>
#include FilIndUSARTDMAHD

UARTDMAPeriph::UARTDMAPeriph(UART_HandleTypeDef *UART_Handle,
                             Form *WrteForm, uint16_t WrteFormSize,
                             Form *ReadForm, uint16_t ReadFormSize,
                             DMA_HandleTypeDef *DMA_Rx_Handle, DMA_HandleTypeDef *DMA_Tx_Handle)
/**************************************************************************************************
 * Create a UARTDMAPeripg class handler. This class is based upon the UARTPeriph adding functions
 * to support interfacing with DMA (DMAPeriph)
 *************************************************************************************************/
: UARTPeriph(UART_Handle, WrteForm, WrteFormSize, ReadForm, ReadFormSize) {

    // Link the DMA handles to the class internals
    this->DMA_Rx = DMA_Rx_Handle;
    this->DMA_Tx = DMA_Tx_Handle;

    // Determine what mode has been selected -
    //  if there is no actual handle provided (NULL) then indicate DMA is disabled
    if (DMA_Rx_Handle == __null) {  this->ModeRx  =  DMAMode::disable;  }
    else                         {  this->ModeRx  =  DMAMode::enable;   };
    if (DMA_Tx_Handle == __null) {  this->ModeTx  =  DMAMode::disable;  }
    else                         {  this->ModeTx  =  DMAMode::enable;   };
}

void UARTDMAPeriph::UARTInterruptStart(void) {
/**************************************************************************************************
 * Function will be called to start off any new UART communication (read/write) if there is
 * anything within either of the queues, and the bus is free.
 *   If the 'UARTDMAPeriph' class has been constructed with a DMA pointer, the below will
 *   utilise the DMA for either Receive/Transmit/both
 *************************************************************************************************/
    // Write Communication setup
    if ( (this->WrteCommState == Free) && (this->FormWrteQ.State() != GenBuffer_Empty) ) {
        // If the UART (write) bus is free, and there is a UART write transmit request forms in
        // the queue
        this->FormWrteQ.OutputRead( &(this->curWrteForm) );     // Capture form request

        // Check current form to see if a fault has already been detected - therefore any new
        // request is no longer valid
        while (  *(this->curWrteForm.Flt) != UARTPeriph::DevFlt::None  ) {
            // If there is a fault in request form, check to see if there is a new request
            if ( this->FormWrteQ.State() == GenBuffer_Empty ) { // If buffer is empty, break out
                //this->Disable();
                return;
            }
            // If there is something in the queue, then make it current. Then re-check
            this->FormWrteQ.OutputRead( &(this->curWrteForm) );     // Capture form request
        }

        this->WrteCommState = Communicating;    // Lock UART bus

        //this->Enable();

        this->curWrteCount  = this->curWrteForm.size;

        // Depending upon the DMA mode
        if (this->ModeTx == DMAMode::disable) {     // If DMA is not enabled, then communication
                                                    // is managed by interrupts, therefore enable
                                                    // interrupts
            this->configTransmtIT(ITEnable);        // Then enable Transmit Empty buffer interrupt
        } //---------------------------------------------------------------------------------------
        else {                                      // If DMA is enabled, then communication is
                                                    // managed by DMA, therefore link requested
                                                    // data to required DMA
            /* Steps to setup Transmit DMA
                1] Disable DMA
                2] Link Target to UART Peripheral register
                3] Link Destination to first array location
                4] Enter number of bytes
                    4a] Mode should be configured via STM32CubeMX
                5] Enable USART Register for DMA Transmit management
                    5a] Clear the Transmit Complete flag, for DMA utilisation
                6] Enable DMA
             */
            __HAL_DMA_DISABLE(this->DMA_Tx);        // Disable DMA
#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
            this->popDMARegisters(this->DMA_Tx,  (uint32_t) this->curWrteForm.Buff,
                                                 (uint32_t)&this->UART_Handle->Instance->DR,
                                                 this->curWrteForm.size);

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================
            this->popDMARegisters(this->DMA_Tx,  (uint32_t) this->curWrteForm.Buff,
                                                 (uint32_t)&this->UART_Handle->Instance->TDR,
                                                 this->curWrteForm.size);

#else
//==================================================================================================

#endif
            SET_BIT(this->UART_Handle->Instance->CR3, USART_CR3_DMAT);  // Link USART to DMA

            // Also enable the Complete Transmission Interrupt
            __HAL_DMA_ENABLE_IT(this->DMA_Tx, DMA_IT_TC);
            __HAL_DMA_ENABLE_IT(this->DMA_Tx, DMA_IT_TE);   // Enable DMA error interrupt

            this->ClearComptChk();

            __HAL_DMA_ENABLE(this->DMA_Tx); // Enable DMA
        }
    }
    else if ( (this->WrteCommState == Free) && (this->FormWrteQ.State() == GenBuffer_Empty) ) {
        //this->Disable();
    }

    // Read Communication setup
    if ( (this->ReadCommState == Free) && (this->FormReadQ.State() != GenBuffer_Empty) ) {
        // If the UART (write) bus is free, and there is a UART write transmit request forms in
        // the queue
        this->FormReadQ.OutputRead( &(this->curReadForm) );     // Capture form request

        // Check current form to see if a fault has already been detected - therefore any new
        // request is no longer valid
        while (  *(this->curReadForm.Flt) != UARTPeriph::DevFlt::None  ) {
            // If there is a fault in request form, check to see if there is a new request
            if ( this->FormReadQ.State() == GenBuffer_Empty ) { // If buffer is empty, break out
                //this->Disable();
                return;
            }
            // If there is something in the queue, then make it current. Then re-check
            this->FormWrteQ.OutputRead( &(this->curReadForm) );     // Capture form request
        }

        this->ReadCommState = Communicating;    // Lock UART bus

        //this->Enable();

        this->curReadCount  = this->curReadForm.size;

        // Depending upon the DMA mode
        if (this->ModeRx == DMAMode::disable) {     // If DMA is not enabled, then communication
                                                    // is managed by interrupts, therefore enable
                                                    // interrupts
            this->configReceiveIT(ITEnable);        // Then enable Receive buffer full interrupt
        } //---------------------------------------------------------------------------------------
        else {                                      // If DMA is enabled, then communication is
                                                    // managed by DMA, therefore link requested
                                                    // data to required DMA
            /* Steps to setup Transmit DMA
                1] Disable DMA
                2] Link Target to UART Peripheral register
                3] Link Destination to first array location
                4] Enter number of bytes
                    4a] Mode should be configured via STM32CubeMX
                5] Enable USART Register for DMA Transmit management
                    5a] Clear the Transmit Complete flag, for DMA utilisation
                6] Enable DMA
             */
            __HAL_DMA_DISABLE(this->DMA_Rx);    // Disable DMA
#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
            this->popDMARegisters(this->DMA_Rx,  (uint32_t)&this->UART_Handle->Instance->DR,
                                                 (uint32_t) this->curReadForm.Buff,
                                                 this->curReadForm.size);

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================
            this->popDMARegisters(this->DMA_Rx,  (uint32_t)&this->UART_Handle->Instance->RDR,
                                                 (uint32_t) this->curReadForm.Buff,
                                                 this->curReadForm.size);

#else
//==================================================================================================

#endif
            SET_BIT(this->UART_Handle->Instance->CR3, USART_CR3_DMAR);  // Link USART to DMA

            // This version assumes that the Recieve DMA is in circular mode, and therefore
            // no interrupt is required, as will ALWAYS push data to the desired array
            //__HAL_DMA_ENABLE_IT(this->DMA_Rx, DMA_IT_TC);
            __HAL_DMA_ENABLE_IT(this->DMA_Rx, DMA_IT_TE);   // Enable DMA error interrupt
            __HAL_DMA_ENABLE(this->DMA_Rx);     // Enable DMA
        }
    }
    else if ( (this->ReadCommState == Free) && (this->FormReadQ.State() == GenBuffer_Empty) ) {
        //this->Disable();
    }
}

void UARTDMAPeriph::intWrteFormCmplt(void) {
/**************************************************************************************************
 * Updates the active form for writing, to indicate how much data has been completed.
 * Indicates that the UART write Device is now free for any new communication
 *   If DMA has been enabled, the DMA will be disabled along with the interrupt flags.
 *
 * The UART transmission interrupt flags (Transmit Empty, and Transmit complete) are all cleared.
 * Other functions will enable whichever is required.
 *************************************************************************************************/
    if (this->ModeTx == DMAMode::disable) {
        *(this->curWrteForm.Cmplt)  += (this->curWrteForm.size - this->curWrteCount);
            // Indicate how many data points have been transfered (curCount should be 0)

    } else {
        *(this->curWrteForm.Cmplt)  +=
                        (this->curWrteForm.size - __HAL_DMA_GET_COUNTER(this->DMA_Tx));
            // Indicate how many data points have been transfered (curCount should be 0)

        __HAL_DMA_DISABLE(this->DMA_Tx);                    // Disable DMA
        __HAL_DMA_DISABLE_IT(this->DMA_Tx, DMA_IT_TC);      // Disable DMA complete interrupt
        __HAL_DMA_DISABLE_IT(this->DMA_Tx, DMA_IT_TE);      // Disable DMA error interrupt
    }

    this->configTransmtIT(ITDisable);       // Disable Transmit empty buffer interrupt
    this->configTransCmIT(ITDisable);       // Disable Transmit complete interrupt

    // Indicate that UART bus is now free, and disable any interrupts
    this->WrteCommState = Free;
}

void UARTDMAPeriph::intReadFormCmplt(void) {
/**************************************************************************************************
 * Updates the active form for reading, to indicate how much data has been completed.
 * Indicates that the UART read Device is now free for any new communication
 *   If DMA has been enabled, the DMA will be disabled along with the interrupt flags.
 *
 * The UART receive interrupt flags will be cleared, as other functions will enable whichever is
 * required.
 *************************************************************************************************/
    if (this->ModeRx == DMAMode::disable) {
        *(this->curReadForm.Cmplt)  += (this->curReadForm.size - this->curReadCount);
            // Indicate how many data points have been transfered (curCount should be 0)

    } else {
        *(this->curReadForm.Cmplt)  +=
                        (this->curReadForm.size - __HAL_DMA_GET_COUNTER(this->DMA_Rx));
            // Indicate how many data points have been transfered (curCount should be 0)

        __HAL_DMA_DISABLE(this->DMA_Rx);                    // Disable DMA
        __HAL_DMA_DISABLE_IT(this->DMA_Rx, DMA_IT_TC);      // Disable DMA complete interrupt
        __HAL_DMA_DISABLE_IT(this->DMA_Rx, DMA_IT_TE);      // Disable DMA error interrupt
    }

    this->configReceiveIT(ITDisable);       // Disable Receive buffer full interrupt

    // Indicate that UART bus is now free, and disable any interrupts
    this->ReadCommState = Free;
}

void UARTDMAPeriph::Read_GenBufferLock(GenBuffer<uint8_t> *ReadArray,
                                    volatile DevFlt *fltReturn, volatile uint16_t *cmpFlag) {
/**************************************************************************************************
 * Intended to allow link to a GenBuffer class, whilst also utilising the UART Form system.
 * Will in essence ensure that there is always a request to read data, and put this into the
 * GenBuffer.
 *   Will check to see if the communication for read becomes "Free" (current read request has
 *   its intended size), will request a new read - pointing to the top of the GenBuffer array, and
 *   enter the size of the array (retrieved from the GenBuffer class). Then provides top level
 *   fault status and completed flags (won't really be used).
 *
 *   Note with the DMA being enabled, it is expected that the DMA would have been configured for
 *   circular use (essentially never completes), so the below re-enabling of DMAs will not be used.
 *   It will however also work if DMA is not in circular mode.
 *
 * Finally, will update the 'input_pointer' of the GenBuffer, so as to align with the current read
 * status.
 *************************************************************************************************/
    if (this->ReadCommState == CommLock::Free) {    // If the UART Receive is free to be used
        *fltReturn = DevFlt::None;                  // Clear the linked fault flag
        cmpFlag = 0;                                // Clear complete flag

        this->intReadPacket( ReadArray->pa, ReadArray->length, fltReturn, cmpFlag);
            // Request a new read back
    }

    if (this->ModeRx == DMAMode::disable) { // If the DMA is disabled then...
        ReadArray->input_pointer = ReadArray->length - this->curReadCount;
        // use internal class structure - 'curReadCount' to calculate how many data points have
        // been read back
    }
    else {                                  // If the DMA is enabled then...
        ReadArray->input_pointer = ReadArray->length - __HAL_DMA_GET_COUNTER(this->DMA_Rx);
        // use DMA count register - 'CNDTR' to calculate how many data points have been read back
    }
}

void UARTDMAPeriph::IRQDMATxHandle(void) {
/**************************************************************************************************
 * INTERRUPTS:
 * Interrupt Service Routine for the UARTDMA class.
 *
 * Function will then read the hardware status flags, and determine which interrupt has been
 * triggered (will only run if the Transmit DMA interrupt has been triggered):
 *      If a DMA error has occurred, then clear the flag, disable DMA (will already be disabled),
 *      and set the fault flag to 'DMA_Tx_Error'. Communication basically halts.
 *
 *      If DMA half complete - not supported
 *
 *      If DMA transfer complete - enable the UART transmission complete interrupt (to then allow
 *      the normal UARTPeriph interrupt to progress communication).
 *      Will also clear the DMA interrupt flag
 *
 *      At the end will use the 'Global' clear interrupt flag for the DMA, to ensure that all
 *      events have been cleared.
 *      No other interrupts are currently supported.
 *************************************************************************************************/
    // Check to see if there are ANY status updates for the DMA
    if (__HAL_DMA_GET_FLAG(this->DMA_Tx, __HAL_DMA_GET_GI_FLAG_INDEX(this->DMA_Tx)) != 0) {
        // If an interrupt has been detected, then...
        // Now check to see which interrupt has been triggered:
        // 1 - Error(s)
        if ( (__HAL_DMA_GET_IT_SOURCE(this->DMA_Tx, DMA_IT_TE) != 0) &&
             (__HAL_DMA_GET_FLAG(this->DMA_Tx, __HAL_DMA_GET_TE_FLAG_INDEX(this->DMA_Tx)) != 0) ) {

            __HAL_DMA_CLEAR_FLAG(this->DMA_Tx, __HAL_DMA_GET_TE_FLAG_INDEX(this->DMA_Tx));
                // Clear the interrupt flag

            __HAL_DMA_DISABLE(this->DMA_Tx);    // This type of error will already have disabled
                                                // DMA, however this is to enforce within code
            *(this->curWrteForm.Flt) = DevFlt::DMA_Tx_Error;    // Indicate fault (Tx_Error)

            // Essentially if there are any DMA error faults, then cancel all Transmissions
        }

        // 2 - Half Transmission - Not utilised within this class set
        // 3 - Full Transmission
        if ( (__HAL_DMA_GET_IT_SOURCE(this->DMA_Tx, DMA_IT_TC) != 0) &&
             (__HAL_DMA_GET_FLAG(this->DMA_Tx, __HAL_DMA_GET_TC_FLAG_INDEX(this->DMA_Tx)) != 0) ) {
            this->configTransCmIT(ITEnable);                // Enable Transmission complete
                                                            // interrupt, such that "IRQHandle"
                                                            // is able to progress transmission
            __HAL_DMA_CLEAR_FLAG(this->DMA_Tx, __HAL_DMA_GET_TC_FLAG_INDEX(this->DMA_Tx));
                // Clear the interrupt flag
        }

        // Use global clear flag, to clear all interrupts (if not already done so)
        __HAL_DMA_CLEAR_FLAG(this->DMA_Tx, __HAL_DMA_GET_GI_FLAG_INDEX(this->DMA_Tx));
    }

}

void UARTDMAPeriph::IRQDMARxHandle(void) {
/**************************************************************************************************
 * INTERRUPTS:
 * Interrupt Service Routine for the UARTDMA class.
 *
 * Function will then read the hardware status flags, and determine which interrupt has been
 * triggered (will only run if the Receive DMA interrupt has been triggered):
 *      If a DMA error has occurred, then clear the flag, disable DMA (will already be disabled),
 *      and set the fault flag to 'DMA_Rx_Error'. Communication basically halts.
 *
 *      If DMA half complete - not supported
 *
 *      If DMA transfer complete - call functions to tidy up Receive forms, and disable DMA. Also
 *      check to see if there is any other requires to read via UART.
 *      Will also clear the DMA interrupt flag
 *
 *      At the end will use the 'Global' clear interrupt flag for the DMA, to ensure that all
 *      events have been cleared.
 *      No other interrupts are currently supported.
 *************************************************************************************************/
    // Check to see if there are ANY status updates for the DMA
    if (__HAL_DMA_GET_FLAG(this->DMA_Rx, __HAL_DMA_GET_GI_FLAG_INDEX(this->DMA_Rx)) != 0) {
        // If an interrupt has been detected, then...
        // Now check to see which interrupt has been triggered:
        // 1 - Error(s)
        if ( (__HAL_DMA_GET_IT_SOURCE(this->DMA_Rx, DMA_IT_TE) != 0) &&
             (__HAL_DMA_GET_FLAG(this->DMA_Rx, __HAL_DMA_GET_TE_FLAG_INDEX(this->DMA_Rx)) != 0) ) {

            __HAL_DMA_CLEAR_FLAG(this->DMA_Rx, __HAL_DMA_GET_TE_FLAG_INDEX(this->DMA_Rx));
                // Clear the interrupt flag

            __HAL_DMA_DISABLE(this->DMA_Rx);    // This type of error will already have disabled
                                                // DMA, however this is to enforce within code
            *(this->curWrteForm.Flt) = DevFlt::DMA_Rx_Error;    // Indicate fault (Rx_Error)

            // Essentially if there are any DMA error faults, then cancel all Transmissions
        }

        // 2 - Half Transmission - Not utilised within this class set
        // 3 - Full Transmission
        if ( (__HAL_DMA_GET_IT_SOURCE(this->DMA_Rx, DMA_IT_TC) != 0) &&
             (__HAL_DMA_GET_FLAG(this->DMA_Rx, __HAL_DMA_GET_TC_FLAG_INDEX(this->DMA_Rx)) != 0) ) {
            __HAL_DMA_CLEAR_FLAG(this->DMA_Rx, __HAL_DMA_GET_TC_FLAG_INDEX(this->DMA_Rx));
                // Clear the interrupt flag

            this->intReadFormCmplt();               // Complete the current request form
                                                    // (no faults)
            this->UARTInterruptStart();             // Check if any new requests
                                                    // remain
        }

        // Use global clear flag, to clear all interrupts (if not already done so)
        __HAL_DMA_CLEAR_FLAG(this->DMA_Rx, __HAL_DMA_GET_GI_FLAG_INDEX(this->DMA_Rx));
    }
}

UARTDMAPeriph::~UARTDMAPeriph() {
/**************************************************************************************************
 * When the destructor is called, need to ensure that the memory allocation is cleaned up, so as
 * to avoid "memory leakage"
 *************************************************************************************************/
    // None
}

