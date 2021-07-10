/**************************************************************************************************
 * @file        UARTDMAPeriph.cpp
 * @author      Thomas
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
    _dma_rx_ = DMA_Rx_Handle;
    _dma_tx_ = DMA_Tx_Handle;

    // Determine what mode has been selected -
    //  if there is no actual handle provided (NULL) then indicate DMA is disabled
    if (DMA_Rx_Handle == __null) {  _mode_rx_  =  DMAMode::kDisable;  }
    else                         {  _mode_rx_  =  DMAMode::kEnable;   };
    if (DMA_Tx_Handle == __null) {  _mode_tx_  =  DMAMode::kDisable;  }
    else                         {  _mode_tx_  =  DMAMode::kEnable;   };
}

void UARTDMAPeriph::startInterrupt(void) {
/**************************************************************************************************
 * Function will be called to start off any new UART communication (read/write) if there is
 * anything within either of the queues, and the bus is free.
 *   If the 'UARTDMAPeriph' class has been constructed with a DMA pointer, the below will
 *   utilise the DMA for either Receive/Transmit/both
 *************************************************************************************************/
    // Write Communication setup
    if ( (wrte_comm_state == CommLock::kFree) && (_form_wrte_q_.state() != kGenBuffer_Empty) ) {
        // If the UART (write) bus is free, and there is a UART write transmit request forms in
        // the queue
        _form_wrte_q_.outputRead( &(_cur_wrte_form_) );     // Capture form request

        // Check current form to see if a fault has already been detected - therefore any new
        // request is no longer valid
        while (  *(_cur_wrte_form_.Flt) != DevFlt::kNone  ) {
            // If there is a fault in request form, check to see if there is a new request
            if ( _form_wrte_q_.state() == kGenBuffer_Empty ) {  // If buffer is empty break out
                //Disable();
                return;
            }
            // If there is something in the queue, then make it current. Then re-check
            _form_wrte_q_.outputRead( &(_cur_wrte_form_) );     // Capture form request
        }

        wrte_comm_state = CommLock::kCommunicating; // Lock UART bus

        //Enable();

        _cur_wrte_count_  = _cur_wrte_form_.size;

        // Depending upon the DMA mode
        if (_mode_tx_ == DMAMode::kDisable) {       // If DMA is not enabled, then communication
                                                    // is managed by interrupts, therefore enable
                                                    // interrupts
            configTransmtIT(InterState::kIT_Enable);// Then enable Transmit Empty buffer interrupt
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

            __HAL_DMA_DISABLE(_dma_tx_);// Disable DMA
#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
            popDMARegisters(_dma_tx_,  (uint32_t) _cur_wrte_form_.Buff,
                                                  (uint32_t)&_uart_handle_->Instance->DR,
                                                  _cur_wrte_form_.size);

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================
            popDMARegisters(_dma_tx_,  (uint32_t) _cur_wrte_form_.Buff,
                                                  (uint32_t)&_uart_handle_->Instance->TDR,
                                                  _cur_wrte_form_.size);

#else
//==================================================================================================

#endif
            SET_BIT(_uart_handle_->Instance->CR3, USART_CR3_DMAT);  // Link USART to DMA

            // Also enable the Complete Transmission Interrupt
            __HAL_DMA_ENABLE_IT(_dma_tx_, DMA_IT_TC);
            __HAL_DMA_ENABLE_IT(_dma_tx_, DMA_IT_TE);   // Enable DMA error interrupt

            clearComptChk();

            __HAL_DMA_ENABLE(_dma_tx_); // Enable DMA
        }
    }
    else if ( (wrte_comm_state == CommLock::kFree) &&
              (_form_wrte_q_.state() == kGenBuffer_Empty) ) {
        //Disable();
    }

    // Read Communication setup
    if ( (read_comm_state == CommLock::kFree) && (_form_read_q_.state() != kGenBuffer_Empty) ) {
        // If the UART (write) bus is free, and there is a UART write transmit request forms in
        // the queue
        _form_read_q_.outputRead( &(_cur_read_form_) );     // Capture form request

        // Check current form to see if a fault has already been detected - therefore any new
        // request is no longer valid
        while (  *(_cur_read_form_.Flt) != DevFlt::kNone  ) {
            // If there is a fault in request form, check to see if there is a new request
            if ( _form_read_q_.state() == kGenBuffer_Empty ) {  // If buffer is empty, break out
                //Disable();
                return;
            }
            // If there is something in the queue, then make it current. Then re-check
            _form_read_q_.outputRead( &(_cur_read_form_) );     // Capture form request
        }

        read_comm_state = CommLock::kCommunicating; // Lock UART bus

        //Enable();

        _cur_read_count_  = _cur_read_form_.size;

        // Depending upon the DMA mode
        if (_mode_rx_ == DMAMode::kDisable) {       // If DMA is not enabled, then communication
                                                    // is managed by interrupts, therefore enable
                                                    // interrupts
            configReceiveIT(InterState::kIT_Enable);// Then enable Receive buffer full interrupt
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
            __HAL_DMA_DISABLE(_dma_rx_);    // Disable DMA
#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
            popDMARegisters(_dma_rx_,  (uint32_t)&_uart_handle_->Instance->DR,
                                                 (uint32_t) _cur_read_form_.Buff,
                                                 _cur_read_form_.size);

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================
            popDMARegisters(_dma_rx_,  (uint32_t)&_uart_handle_->Instance->RDR,
                                                 (uint32_t) _cur_read_form_.Buff,
                                                 _cur_read_form_.size);

#else
//==================================================================================================

#endif
            SET_BIT(_uart_handle_->Instance->CR3, USART_CR3_DMAR);  // Link USART to DMA

            // This version assumes that the Recieve DMA is in circular mode, and therefore
            // no interrupt is required, as will ALWAYS push data to the desired array
            //__HAL_DMA_ENABLE_IT(DMA_Rx, DMA_IT_TC);
            __HAL_DMA_ENABLE_IT(_dma_rx_, DMA_IT_TE);   // Enable DMA error interrupt
            __HAL_DMA_ENABLE(_dma_rx_);     // Enable DMA
        }
    }
    else if ( (read_comm_state == CommLock::kFree) &&
              (_form_read_q_.state() == kGenBuffer_Empty) ) {
        //Disable();
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
    if (_mode_tx_ == DMAMode::kDisable) {
        *(_cur_wrte_form_.Cmplt)  += (_cur_wrte_form_.size - _cur_wrte_count_);
            // Indicate how many data points have been transfered (curCount should be 0)

    } else {
        *(_cur_wrte_form_.Cmplt)  +=
                        (_cur_wrte_form_.size - __HAL_DMA_GET_COUNTER(_dma_tx_));
            // Indicate how many data points have been transfered (curCount should be 0)

        __HAL_DMA_DISABLE(_dma_tx_);                // Disable DMA
        __HAL_DMA_DISABLE_IT(_dma_tx_, DMA_IT_TC);  // Disable DMA complete interrupt
        __HAL_DMA_DISABLE_IT(_dma_tx_, DMA_IT_TE);  // Disable DMA error interrupt
    }

    configTransmtIT(InterState::kIT_Disable);       // Disable Transmit empty buffer interrupt
    configTransCmIT(InterState::kIT_Disable);       // Disable Transmit complete interrupt

    // Indicate that UART bus is now free, and disable any interrupts
    wrte_comm_state = CommLock::kFree;
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
    if (_mode_rx_ == DMAMode::kDisable) {
        *(_cur_read_form_.Cmplt)  += (_cur_read_form_.size - _cur_read_count_);
            // Indicate how many data points have been transfered (curCount should be 0)

    } else {
        *(_cur_read_form_.Cmplt)  +=
                        (_cur_read_form_.size - __HAL_DMA_GET_COUNTER(_dma_rx_));
            // Indicate how many data points have been transfered (curCount should be 0)

        __HAL_DMA_DISABLE(_dma_rx_);                // Disable DMA
        __HAL_DMA_DISABLE_IT(_dma_rx_, DMA_IT_TC);  // Disable DMA complete interrupt
        __HAL_DMA_DISABLE_IT(_dma_rx_, DMA_IT_TE);  // Disable DMA error interrupt
    }

    configReceiveIT(InterState::kIT_Disable);       // Disable Receive buffer full interrupt

    // Indicate that UART bus is now free, and disable any interrupts
    read_comm_state = CommLock::kFree;
}

void UARTDMAPeriph::readGenBufferLock(GenBuffer<uint8_t> *ReadArray,
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
    if (read_comm_state == CommLock::kFree) {       // If the UART Receive is free to be used
        *fltReturn = DevFlt::kNone;                 // Clear the linked fault flag
        cmpFlag = 0;                                // Clear complete flag

        intReadPacket( ReadArray->pa, ReadArray->length, fltReturn, cmpFlag);
            // Request a new read back
    }

    if (_mode_rx_ == DMAMode::kDisable) {   // If the DMA is disabled then...
        ReadArray->input_pointer = ReadArray->length - _cur_read_count_;
        // use internal class structure - 'curReadCount' to calculate how many data points have
        // been read back
    }
    else {                                  // If the DMA is enabled then...
        ReadArray->input_pointer = ReadArray->length - __HAL_DMA_GET_COUNTER(_dma_rx_);
        // use DMA count register - 'CNDTR' to calculate how many data points have been read back
    }
}

void UARTDMAPeriph::handleDMATxIRQ(void) {
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
    if (__HAL_DMA_GET_FLAG(_dma_tx_, __HAL_DMA_GET_GI_FLAG_INDEX(_dma_tx_)) != 0) {
        // If an interrupt has been detected, then...
        // Now check to see which interrupt has been triggered:
        // 1 - Error(s)
        if ( (__HAL_DMA_GET_IT_SOURCE(_dma_tx_, DMA_IT_TE) != 0) &&
             (__HAL_DMA_GET_FLAG(_dma_tx_, __HAL_DMA_GET_TE_FLAG_INDEX(_dma_tx_)) != 0) ) {

            __HAL_DMA_CLEAR_FLAG(_dma_tx_, __HAL_DMA_GET_TE_FLAG_INDEX(_dma_tx_));
                // Clear the interrupt flag

            __HAL_DMA_DISABLE(_dma_tx_);    // This type of error will already have disabled
                                            // DMA, however this is to enforce within code
            *(_cur_wrte_form_.Flt) = DevFlt::kDMA_Tx_Error; // Indicate fault (Tx_Error)

            // Essentially if there are any DMA error faults, then cancel all Transmissions
        }

        // 2 - Half Transmission - Not utilised within this class set
        // 3 - Full Transmission
        if ( (__HAL_DMA_GET_IT_SOURCE(_dma_tx_, DMA_IT_TC) != 0) &&
             (__HAL_DMA_GET_FLAG(_dma_tx_, __HAL_DMA_GET_TC_FLAG_INDEX(_dma_tx_)) != 0) ) {
            configTransCmIT(InterState::kIT_Enable);        // Enable Transmission complete
                                                            // interrupt, such that "IRQHandle"
                                                            // is able to progress transmission
            __HAL_DMA_CLEAR_FLAG(_dma_tx_, __HAL_DMA_GET_TC_FLAG_INDEX(_dma_tx_));
                // Clear the interrupt flag
        }

        // Use global clear flag, to clear all interrupts (if not already done so)
        __HAL_DMA_CLEAR_FLAG(_dma_tx_, __HAL_DMA_GET_GI_FLAG_INDEX(_dma_tx_));
    }

}

void UARTDMAPeriph::handleDMARxIRQ(void) {
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
    if (__HAL_DMA_GET_FLAG(_dma_rx_, __HAL_DMA_GET_GI_FLAG_INDEX(_dma_rx_)) != 0) {
        // If an interrupt has been detected, then...
        // Now check to see which interrupt has been triggered:
        // 1 - Error(s)
        if ( (__HAL_DMA_GET_IT_SOURCE(_dma_rx_, DMA_IT_TE) != 0) &&
             (__HAL_DMA_GET_FLAG(_dma_rx_, __HAL_DMA_GET_TE_FLAG_INDEX(_dma_rx_)) != 0) ) {

            __HAL_DMA_CLEAR_FLAG(_dma_rx_, __HAL_DMA_GET_TE_FLAG_INDEX(_dma_rx_));
                // Clear the interrupt flag

            __HAL_DMA_DISABLE(_dma_rx_);    // This type of error will already have disabled
                                            // DMA, however this is to enforce within code
            *(_cur_wrte_form_.Flt) = DevFlt::kDMA_Rx_Error; // Indicate fault (Rx_Error)

            // Essentially if there are any DMA error faults, then cancel all Transmissions
        }

        // 2 - Half Transmission - Not utilised within this class set
        // 3 - Full Transmission
        if ( (__HAL_DMA_GET_IT_SOURCE(_dma_rx_, DMA_IT_TC) != 0) &&
             (__HAL_DMA_GET_FLAG(_dma_rx_, __HAL_DMA_GET_TC_FLAG_INDEX(_dma_rx_)) != 0) ) {
            __HAL_DMA_CLEAR_FLAG(_dma_rx_, __HAL_DMA_GET_TC_FLAG_INDEX(_dma_rx_));
                // Clear the interrupt flag

            intReadFormCmplt(); // Complete the current request form (no faults)
            startInterrupt();   // Check if any new requests remain
        }

        // Use global clear flag, to clear all interrupts (if not already done so)
        __HAL_DMA_CLEAR_FLAG(_dma_rx_, __HAL_DMA_GET_GI_FLAG_INDEX(_dma_rx_));
    }
}

UARTDMAPeriph::~UARTDMAPeriph() {
/**************************************************************************************************
 * When the destructor is called, need to ensure that the memory allocation is cleaned up, so as
 * to avoid "memory leakage"
 *************************************************************************************************/
    // None
}

