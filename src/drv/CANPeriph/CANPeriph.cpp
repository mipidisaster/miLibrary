/**************************************************************************************************
 * @file        CANPeriph.cpp
 * @author      Thomas
 * @brief       Source file for the Generic CANPeriph Class handle
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include <FileIndex.h>                  // Header for miLibrary index
#include FilInd_CANPe__HD               // Header for CAN

// C System Header(s)
// ------------------
#include <stdint.h>

// C++ System Header(s)
// --------------------
// None

// Other Libraries
// --------------
#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
#include "stm32f1xx_hal.h"              // Include the HAL UART library

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
#include "stm32l4xx_hal.h"              // Include the HAL UART library

#elif (zz__MiEmbedType__zz == 10)       // If the target device is an Raspberry Pi then
//=================================================================================================
#error "No CAN interface for RaspberryPi"

#elif (defined(zz__MiEmbedType__zz)) && (zz__MiEmbedType__zz ==  0)
//     If using the Linux (No Hardware) version then
//=================================================================================================
#error "Linux dummy interface not supported"

#else
//=================================================================================================
#error "Unrecognised target device"

#endif

// Project Libraries
// -----------------
#include FilInd_GENBUF_TP               // Provide the template for the circular buffer class

//=================================================================================================

#define CANPERIPH_RX        1
#define CANPERIPH_TX        0

void CANPeriph::popGenParam(void) {
/**************************************************************************************************
 * Generate default parameters for the CANPeriph class. To be called by all constructors.
 * Initial construction will populate the internal GenBuffers with default parameters (as basic
 * constructor of GenBuffer is already set to zero).
 *************************************************************************************************/
    tx_flt[0]       = DevFlt::kInitialised; // Initialise the fault to "initialised"
    tx_flt[1]       = DevFlt::kInitialised; //
    tx_flt[2]       = DevFlt::kInitialised; //
    rx_flt[0]       = DevFlt::kInitialised; //
    rx_flt[1]       = DevFlt::kInitialised; //

    wrte_error_count = 0;
    read_error_count = 0;
}

#if   ( (zz__MiEmbedType__zz == 50) || (zz__MiEmbedType__zz == 51)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================
CANPeriph::CANPeriph(CAN_HandleTypeDef *CAN_Handle,
                     Form *WrteForm, uint16_t WrteFormSize,
                     Form *ReadForm, uint16_t ReadFormSize) {
/**************************************************************************************************
 * Create a CANPeriph class specific for the STM32 device
 * Receives the address of the CAN Handle of device - generated from cubeMX
 *
 * As the STM32CubeMX already pre-generates the setting up and configuring of the desired CAN
 * device, there is no need to define that within this function. Simply providing the handle is
 * required.
 *************************************************************************************************/
    popGenParam();                      // Populate generic class parameters
    _can_handle_ = CAN_Handle;          // Copy data into class

    _form_wrte_q_.create(WrteForm, WrteFormSize);
    _form_read_q_.create(ReadForm, ReadFormSize);
}

#else   // Raspberry Pi or Default build configuration
//=================================================================================================
CANPeriph::CANPeriph() {
/**************************************************************************************************
 * Create a CANPeriph class specific for the STM32 device
 *************************************************************************************************/
    popGenParam();
}

#endif

CANPeriph::DevFlt CANPeriph:: getFaultStatus(uint8_t is_read, uint8_t FIFO) {
/**************************************************************************************************
 * Reads the state of the hardware register storing the fault status.
 * Updates the following internal parameters:
 *      'flt', 'wrte_error_count' and 'read_error_count'
 *
 * Returns the determined fault state of the bus, and will store this in the respective fault
 * status;
 *      is_read = 0 (i.e. write), FIFO will be for the tx_flt flag
 *      is_read = 1 (i.e. read),  FIFO will be for the rx_flt flag
 *************************************************************************************************/
    uint32_t error_register = _can_handle_->Instance->ESR;

    read_error_count = (uint8_t) ( (error_register & 0xFF000000) >> 24 );
    wrte_error_count = (uint8_t) ( (error_register & 0x00FF0000) >> 16 );

    // Will capture the last error code
    DevFlt last_error_count = (DevFlt) ( (error_register & 0x00000070) >> 4);

    if (is_read != CANPERIPH_TX) {
        rx_flt[FIFO] = last_error_count;
    }
    else {
        tx_flt[FIFO] = last_error_count;
    }

    return (last_error_count);
}

uint8_t CANPeriph::receiveOverRunChk(uint8_t RxFIFO) {
/**************************************************************************************************
 * Check to see if the Receive Overrun bit has been set or not.
 *************************************************************************************************/
    uint32_t rx_FIFO_interrupt_register = 0;
    if (RxFIFO == 0)
        rx_FIFO_interrupt_register = CAN_FLAG_FOV0;
    else
        rx_FIFO_interrupt_register = CAN_FLAG_FOV1;

    if ( (__HAL_CAN_GET_FLAG(_can_handle_, rx_FIFO_interrupt_register)   != 0 ) )
        return (1);
    else
        return (0);
}

uint8_t CANPeriph::receiveFullChk(uint8_t RxFIFO) {
/**************************************************************************************************
 * Check to see if the Receive Full bit has been set or not.
 *************************************************************************************************/
    uint32_t rx_FIFO_interrupt_register = 0;
    if (RxFIFO == 0)
        rx_FIFO_interrupt_register = CAN_FLAG_FF0;
    else
        rx_FIFO_interrupt_register = CAN_FLAG_FF1;

    if ( (__HAL_CAN_GET_FLAG(_can_handle_, rx_FIFO_interrupt_register)   != 0 ) )
        return (1);
    else
        return (0);
}

uint8_t CANPeriph::transmitEmptyChk(void) {
/**************************************************************************************************
 * Returns the combined state of the 3 TxFIFOs within the hardware - i.e if the returned value is
 * 7, then ALL mailboxes are free.
 *************************************************************************************************/
    uint8_t transmit_state = 0;

    if ( (__HAL_CAN_GET_FLAG(_can_handle_, CAN_FLAG_RQCP0)   != 0 ) )
        transmit_state |= 0x01;

    if ( (__HAL_CAN_GET_FLAG(_can_handle_, CAN_FLAG_RQCP1)   != 0 ) )
        transmit_state |= 0x02;

    if ( (__HAL_CAN_GET_FLAG(_can_handle_, CAN_FLAG_RQCP2)   != 0 ) )
        transmit_state |= 0x04;

    return ( transmit_state );
}

uint8_t CANPeriph::receivePendingITChk(uint8_t RxFIFO) {
/**************************************************************************************************
 * Check to see if the new message reception interrupt is enabled; where RxFIFO is the FIFO for
 * which the interrupt is checked for.
 *************************************************************************************************/
    uint32_t rx_FIFO_interrupt_register = 0;
    if (RxFIFO == 0)
        rx_FIFO_interrupt_register = CAN_IT_RX_FIFO0_MSG_PENDING;
    else
        rx_FIFO_interrupt_register = CAN_IT_RX_FIFO1_MSG_PENDING;

    if (__HAL_CAN_GET_IT_SOURCE(_can_handle_, rx_FIFO_interrupt_register) != 0 )
        return (1);
    else
        return (0);
}

uint8_t CANPeriph::receiveOverRunITChk(uint8_t RxFIFO) {
/**************************************************************************************************
 * Check to see if the Rx Overrun condition interrupt is enabled; where RxFIFO is the FIFO for
 * which the interrupt is checked for.
 *************************************************************************************************/
    uint32_t rx_FIFO_interrupt_register = 0;
    if (RxFIFO == 0)
        rx_FIFO_interrupt_register = CAN_IT_RX_FIFO0_OVERRUN;
    else
        rx_FIFO_interrupt_register = CAN_IT_RX_FIFO1_OVERRUN;

    if (__HAL_CAN_GET_IT_SOURCE(_can_handle_, rx_FIFO_interrupt_register) != 0 )
        return (1);
    else
        return (0);
}

uint8_t CANPeriph::receiveFullITChk   (uint8_t RxFIFO) {
/**************************************************************************************************
 * Check to see if the Rx full condition interrupt is enabled; where RxFIFO is the FIFO for which
 * the interrupt is checked for.
 *************************************************************************************************/
    uint32_t rx_FIFO_interrupt_register = 0;
    if (RxFIFO == 0)
        rx_FIFO_interrupt_register = CAN_IT_RX_FIFO0_FULL;
    else
        rx_FIFO_interrupt_register = CAN_IT_RX_FIFO1_FULL;

    if (__HAL_CAN_GET_IT_SOURCE(_can_handle_, rx_FIFO_interrupt_register) != 0 )
        return (1);
    else
        return (0);
}

uint8_t CANPeriph::transmitEmptyITChk(void) {
/**************************************************************************************************
 * Check to see if the Tx FIFO empty is enabled; where RxFIFO is the FIFO for which
 * the interrupt is checked for.
 *************************************************************************************************/
    if (__HAL_CAN_GET_IT_SOURCE(_can_handle_, CAN_IT_TX_MAILBOX_EMPTY) != 0 )
        return (1);
    else
        return (0);
}

void CANPeriph::configFilter(uint8_t filter_bank, FiltMode mode, FiltScale scale, uint8_t RxFIFO,
                             FiltEnb filter_enable, uint32_t fr1, uint32_t fr2) {
/**************************************************************************************************
 * Populate CAN filter as per inputs
 * Layout of the fr1 and fr2 parameters is done outside of the function
 *************************************************************************************************/
    SET_BIT(_can_handle_->Instance->FMR, CAN_FMR_FINIT);    // Put all filters into initialisation

    // Find the bit position of the selected filter bank, for enabling/disabling/etc.
    uint32_t filternbrbitpos = (uint32_t)1 << (filter_bank & 0x1FU);

    // Clear the activation bit - such that the selected filter bank is disabled
    CLEAR_BIT(_can_handle_->Instance->FA1R, filternbrbitpos);

    if (scale  ==  FiltScale::k16bit)   // 16bit mode enabled
        /* 16-bit scale for the filter */
        CLEAR_BIT(_can_handle_->Instance->FS1R, filternbrbitpos);

    else                                // 32bit mode enabled
        /* 32-bit scale for the filter */
        SET_BIT(_can_handle_->Instance->FS1R, filternbrbitpos);

    _can_handle_->Instance->sFilterRegister[filter_bank].FR1 = fr1;
    _can_handle_->Instance->sFilterRegister[filter_bank].FR2 = fr2;

    if (mode   == FiltMode::KID_Mask) { // Identifier Mask Mode
        CLEAR_BIT(_can_handle_->Instance->FM1R, filternbrbitpos);   // Configure for Identifier
    }                                                               // Mask Mode
    else {                              // Identifier List Mode
        SET_BIT  (_can_handle_->Instance->FM1R, filternbrbitpos);   // Configure for Identifier
    }                                                               // List Mode

    if (RxFIFO == 0) {                  // RxFIFO 0 has been selected
        CLEAR_BIT(_can_handle_->Instance->FFA1R, filternbrbitpos);
    }
    else {                              // RxFIFO 1 has been selected
        SET_BIT  (_can_handle_->Instance->FFA1R, filternbrbitpos);
    }

    if (filter_enable == FiltEnb::kFilt_Enable) {
      SET_BIT   (_can_handle_->Instance->FA1R, filternbrbitpos);
    }

    CLEAR_BIT(_can_handle_->Instance->FMR, CAN_FMR_FINIT);
}

void CANPeriph::config32bitFilter(uint8_t filter_bank, FiltMode mode, uint8_t RxFIFO,
                                  FiltEnb filter_enable,
                                  uint32_t id, uint32_t mask) {
/**************************************************************************************************
 * Construct a 32bit filter
 *************************************************************************************************/
    configFilter(filter_bank, mode, FiltScale::k32bit, RxFIFO, filter_enable, id, mask);
}

void CANPeriph::config16bitFilter(uint8_t filter_bank, FiltMode mode, uint8_t RxFIFO,
                                  FiltEnb filter_enable,
                                  uint16_t mask_id_high,  uint16_t id_high,
                                  uint16_t mask_id_low,   uint16_t id_low) {
/**************************************************************************************************
 * Construct a 32bit filter
 *************************************************************************************************/
    uint32_t fr1 = 0, fr2 = 0;

    // Combine together the first Identifier and Mask. Or the first and second identifier
    fr1 = ( (uint32_t) ((mask_id_high) << 16U) |
            (uint32_t) id_high);

    // Combine together the second Identifier and Mask. Or the third and fourth identifier
    fr2 = ( (uint32_t) ((mask_id_low ) << 16U) |
            (uint32_t) id_low );

    configFilter(filter_bank, mode, FiltScale::k16bit, RxFIFO, filter_enable, fr1, fr2);
}

uint16_t CANPeriph::filterID16bit(uint32_t identifier) {
/**************************************************************************************************
 * Function will read in the input identifier, and construct a filter entry with the filters
 * configured for 16bits.
 * Note - will ALWAYS put the Remote Data entry as '0'
 *************************************************************************************************/
    uint16_t return_entry = 0;      // Initialise variable to contain the output of function

    if ( (identifier & kExtendedFrame) == kExtendedFrame) {     // If Extended
        // update the input 'identifier' such that this parameter is now cleared
        identifier &= ~kExtendedFrame;
        identifier &= 0x1FFFFFFF;   // Ensure only valid IDs are captured
        identifier >>= 15;      // Shift the data down by 15 bits, so that only the standard
                                // identifier and last 3 MSB of extended are remaining

        return_entry |= (uint16_t) ( (identifier) & 0x00000007);    // Mask these 3 MSB
        return_entry |= 0x0008; // Set the 'Extended bit entry'

        identifier >>= 3;       // Shift the identifier down by another 3 (total of 18), to get
    }                           // the standard identifier only

    identifier &= 0x7FF;
    return_entry |= (uint16_t) (identifier << 5);
    return (return_entry);
}

uint16_t CANPeriph::filterMASK16bit(uint32_t identifier) {
/**************************************************************************************************
 * Function will read in the input identifier, and construct a filter MASK entry with the filters
 * configured for 16bits.
 * Note - Will always put the Extended ID and Remote bits to as "Must Match" (to the externally
 *        input ID)
 *************************************************************************************************/
    uint16_t return_entry = filterID16bit(identifier);  // Initialise with standard layout

    return_entry |= 0x0008;     // Force the External ID, to be "Must Match"
    return_entry |= 0x0010;     // Force the Remote Message, to be "Must Match"


    return (return_entry);
}

uint32_t CANPeriph::filterID32bit(uint32_t identifier) {
/**************************************************************************************************
 * Function will read in the input identifier, and construct a filter entry with the filters
 * configured for 32bits.
 * Note - will ALWAYS put the Remote Data entry as '0'
 *************************************************************************************************/
    uint32_t return_entry = 0;      // Initialise variable to contain the output of function

    if ( (identifier & kExtendedFrame) == kExtendedFrame) {     // If Extended
        // update the input 'identifier' such that this parameter is now cleared
        identifier &= ~kExtendedFrame;
        identifier &= 0x1FFFFFFF;   // Ensure only valid IDs are captured

        return_entry |= (uint32_t) (identifier << 3);
        return_entry |= 0x00000004; // Set the 'Extended bit entry'
    }
    else {
        identifier &= 0x7FF;

        return_entry |= (uint32_t) (identifier << (18 + 3));
    }

    return (return_entry);
}

uint32_t CANPeriph::filterMASK32bit(uint32_t identifier) {
/**************************************************************************************************
 * Function will read in the input identifier, and construct a filter MASK entry with the filters
 * configured for 32bits.
 * Note - Will always put the Extended ID and Remote bits to as "Must Match" (to the externally
 *        input ID)
 *************************************************************************************************/
    uint32_t return_entry = filterID32bit(identifier);  // Initialise with standard layout

    return_entry |= 0x00000004; // Force the External ID, to be "Must Match"
    return_entry |= 0x00000002; // Force the Remote Message, to be "Must Match"

    return (return_entry);
}

void CANPeriph::fillTransmitMailBox(Form can_message, uint8_t mailbox_number) {
/**************************************************************************************************
 * Populate the transmit mailbox. The mailbox to be populated is 'mailbox_number'
 *************************************************************************************************/
    uint8_t rtr = 0;
    if (can_message.type == RemTrans::kRemoteFrame) {
        rtr = CAN_RTR_REMOTE;
    }


    if ( (can_message.Identifier & kExtendedFrame) == kExtendedFrame) {     // If Extended
        _can_handle_->Instance->sTxMailBox[mailbox_number].TIR =
                ( (can_message.Identifier & 0x1FFFFFFF) << CAN_TI0R_EXID_Pos) |
                CAN_ID_EXT |
                rtr;
    }
    else {                                                                  // If Standard
        _can_handle_->Instance->sTxMailBox[mailbox_number].TIR =
                ( (can_message.Identifier & 0x7FF)      << CAN_TI0R_STID_Pos) |
                rtr;
    }

    _can_handle_->Instance->sTxMailBox[mailbox_number].TDTR = can_message.size;

    // Clear the mailbox entries
    _can_handle_->Instance->sTxMailBox[mailbox_number].TDHR =
            ((uint32_t)can_message.data[7] << CAN_TDH0R_DATA7_Pos) |
            ((uint32_t)can_message.data[6] << CAN_TDH0R_DATA6_Pos) |
            ((uint32_t)can_message.data[5] << CAN_TDH0R_DATA5_Pos) |
            ((uint32_t)can_message.data[4] << CAN_TDH0R_DATA4_Pos);

    _can_handle_->Instance->sTxMailBox[mailbox_number].TDLR =
            ((uint32_t)can_message.data[3] << CAN_TDL0R_DATA3_Pos) |
            ((uint32_t)can_message.data[2] << CAN_TDL0R_DATA2_Pos) |
            ((uint32_t)can_message.data[1] << CAN_TDL0R_DATA1_Pos) |
            ((uint32_t)can_message.data[0] << CAN_TDL0R_DATA0_Pos);
}

uint8_t CANPeriph::getFreeTxMailbox(void) {
/**************************************************************************************************
 * Retrieve the next free Mailbox
 *  Note, Output will saturate at 2, and override if the Tx FIFO is fully populated
 *************************************************************************************************/
    return ( (_can_handle_->Instance->TSR & CAN_TSR_CODE) >> CAN_TSR_CODE_Pos );
}

uint8_t CANPeriph::getTxMailboxesFreeLevel(void) {
/**************************************************************************************************
 * Count the number of Mail Boxes which are free and ready to be populated.
 *************************************************************************************************/
    uint8_t freelevel = 0U;

    if ((_can_handle_->Instance->TSR & CAN_TSR_TME0) != 0U)
        freelevel++;

    if ((_can_handle_->Instance->TSR & CAN_TSR_TME1) != 0U)
        freelevel++;

    if ((_can_handle_->Instance->TSR & CAN_TSR_TME2) != 0U)
        freelevel++;

    return freelevel;
}

uint8_t CANPeriph::getTxRequestComplete(uint8_t mailbox_number) {
/**************************************************************************************************
 * Check to see if the Mailbox communication has been completed
 *************************************************************************************************/
    uint32_t tx_FIFO_interrupt_register = 0;

    switch (mailbox_number) {
        case 1:
            tx_FIFO_interrupt_register = CAN_FLAG_RQCP1;
            break;

        case 2:
            tx_FIFO_interrupt_register = CAN_FLAG_RQCP2;
            break;

        default:
            tx_FIFO_interrupt_register = CAN_FLAG_RQCP0;
            break;
    }

    if ( (__HAL_CAN_GET_FLAG(_can_handle_, tx_FIFO_interrupt_register)   != 0 ) )
        return (1);
    else
        return (0);
}

void CANPeriph::requestTxMailboxTransmission(uint8_t mailbox_number) {
/**************************************************************************************************
 * Request that the Transmit Mailbox be communicated
 *************************************************************************************************/
    SET_BIT(_can_handle_->Instance->sTxMailBox[mailbox_number].TIR, CAN_TI0R_TXRQ);
}

void CANPeriph::abortTxMailbox(uint8_t mailbox_bitpack_number) {
/**************************************************************************************************
 * Abort the communication of input mailbox number.
 *   Note input number is a bit packed array, were bit 1 = mailbox0, bit 2 = mailbox1, and
 *        bit 3 = mailbox2
 *************************************************************************************************/
    /* Check Tx Mailbox 0 */
    if ((mailbox_bitpack_number & CAN_TX_MAILBOX0) != 0U)
    {
      /* Add cancellation request for Tx Mailbox 0 */
      SET_BIT(_can_handle_->Instance->TSR, CAN_TSR_ABRQ0);
    }

    /* Check Tx Mailbox 1 */
    if ((mailbox_bitpack_number & CAN_TX_MAILBOX1) != 0U)
    {
      /* Add cancellation request for Tx Mailbox 1 */
      SET_BIT(_can_handle_->Instance->TSR, CAN_TSR_ABRQ1);
    }

    /* Check Tx Mailbox 2 */
    if ((mailbox_bitpack_number & CAN_TX_MAILBOX2) != 0U)
    {
      /* Add cancellation request for Tx Mailbox 2 */
      SET_BIT(_can_handle_->Instance->TSR, CAN_TSR_ABRQ2);
    }
}

CANPeriph::Form  CANPeriph::getRxFIFOMessage(uint8_t RxFIFO) {
/**************************************************************************************************
 * Retrieve the contents of the RxFIFO input
 *************************************************************************************************/
    Form        temp = { 0 };

    if ( (CAN_RI0R_IDE & _can_handle_->Instance->sFIFOMailBox[RxFIFO].RIR ) == CAN_ID_STD) {
        temp.Identifier =
            (CAN_RI0R_STID & _can_handle_->Instance->sFIFOMailBox[RxFIFO].RIR)
                >> CAN_TI0R_STID_Pos;
    }
    else {
        temp.Identifier =
            ((CAN_RI0R_EXID | CAN_RI0R_STID) & _can_handle_->Instance->sFIFOMailBox[RxFIFO].RIR)
                >> CAN_RI0R_EXID_Pos;
    }

    if ( (CAN_RI0R_RTR & _can_handle_->Instance->sFIFOMailBox[RxFIFO].RIR) == CAN_RI0R_RTR ) {
        temp.type = RemTrans::kRemoteFrame;
    }
    else {
        temp.type = RemTrans::kDataFrame;
    }

    temp.size = (uint8_t) (
        (CAN_RDT0R_DLC & _can_handle_->Instance->sFIFOMailBox[RxFIFO].RDTR) >> CAN_RDT0R_DLC_Pos
    );

    temp.data[0] = (uint8_t)((CAN_RDL0R_DATA0 & _can_handle_->Instance->sFIFOMailBox[RxFIFO].RDLR)
                >> CAN_RDL0R_DATA0_Pos);
    temp.data[1] = (uint8_t)((CAN_RDL0R_DATA1 & _can_handle_->Instance->sFIFOMailBox[RxFIFO].RDLR)
                >> CAN_RDL0R_DATA1_Pos);
    temp.data[2] = (uint8_t)((CAN_RDL0R_DATA2 & _can_handle_->Instance->sFIFOMailBox[RxFIFO].RDLR)
                >> CAN_RDL0R_DATA2_Pos);
    temp.data[3] = (uint8_t)((CAN_RDL0R_DATA3 & _can_handle_->Instance->sFIFOMailBox[RxFIFO].RDLR)
                >> CAN_RDL0R_DATA3_Pos);
    temp.data[4] = (uint8_t)((CAN_RDH0R_DATA4 & _can_handle_->Instance->sFIFOMailBox[RxFIFO].RDHR)
                >> CAN_RDH0R_DATA4_Pos);
    temp.data[5] = (uint8_t)((CAN_RDH0R_DATA5 & _can_handle_->Instance->sFIFOMailBox[RxFIFO].RDHR)
                >> CAN_RDH0R_DATA5_Pos);
    temp.data[6] = (uint8_t)((CAN_RDH0R_DATA6 & _can_handle_->Instance->sFIFOMailBox[RxFIFO].RDHR)
                >> CAN_RDH0R_DATA6_Pos);
    temp.data[7] = (uint8_t)((CAN_RDH0R_DATA7 & _can_handle_->Instance->sFIFOMailBox[RxFIFO].RDHR)
                >> CAN_RDH0R_DATA7_Pos);

    return (temp);
}

void CANPeriph::releaseRxFIFOMessage(uint8_t RxFIFO) {
/**************************************************************************************************
 * Release the specified FIFO Receive input
 *************************************************************************************************/
    if (RxFIFO == CAN_RX_FIFO0) /* Rx element is assigned to Rx FIFO 0 */
    {
      /* Release RX FIFO 0 */
      SET_BIT(_can_handle_->Instance->RF0R, CAN_RF0R_RFOM0);
    }
    else /* Rx element is assigned to Rx FIFO 1 */
    {
      /* Release RX FIFO 1 */
      SET_BIT(_can_handle_->Instance->RF1R, CAN_RF1R_RFOM1);
    }
}

uint8_t CANPeriph::getRxFIFOFillLevel(uint8_t RxFIFO) {
/**************************************************************************************************
 * Will return the number of messages stored in the selected Rx FIFO ('RxFIFO').
 * Expected to return a value of either 0 to 3; 0 is FIFO empty
 *************************************************************************************************/
    uint8_t filllevel = 0;

    if (RxFIFO == CAN_RX_FIFO0)
    {
      filllevel = (uint8_t) (_can_handle_->Instance->RF0R & CAN_RF0R_FMP0);
    }
    else /* RxFIFO == CAN_RX_FIFO1 */
    {
      filllevel = (uint8_t) (_can_handle_->Instance->RF1R & CAN_RF1R_FMP1);
    }

    /* Return Rx FIFO fill level */
    return filllevel;
}

CANPeriph::DevFlt CANPeriph::poleAddTxDataMessage(uint32_t identifier, uint8_t *wData,
                                                   uint8_t size) {
/**************************************************************************************************
 * Populate the transmit mailbox. With a Data Frame request, will wait indefinitely until there is
 * space in the buffer to transmit the data.
 *************************************************************************************************/
    Form data_frame = { 0 };

    if ( (size > 8) || (size == 0) ) {      // If there is no data, or is greater then amount
                                            // allowed
        return (DevFlt::kData_Error);       // return error with DATA (also update fault state)
    }

    data_frame.Identifier   = identifier;
    data_frame.type         = RemTrans::kDataFrame;
    data_frame.size         = size;

    for (uint8_t i = 0; i != size; i++) {
        data_frame.data[i] = wData[i];
    }

    // Wait until at least one of the mailboxes is available
    while( ((_can_handle_->Instance->TSR & CAN_TSR_TME0) == 0U) &&
           ((_can_handle_->Instance->TSR & CAN_TSR_TME1) == 0U) &&
           ((_can_handle_->Instance->TSR & CAN_TSR_TME2) == 0U) )
    {}

    // Determine which mailbox is available to be populated
    uint8_t transmitmailbox = getFreeTxMailbox();

    // Check transmit mailbox value, if this ever exceeds 2 then just loop.
    //   Note, this shouldn't ever happen, this has been copied from the STM32 code - as protection
    while(transmitmailbox > 2) {
        transmitmailbox = getFreeTxMailbox();
    }

    fillTransmitMailBox(data_frame, transmitmailbox);
    requestTxMailboxTransmission(transmitmailbox);

    while(getTxRequestComplete(transmitmailbox) != 0) {
        getFaultStatus(CANPERIPH_TX, transmitmailbox);

        if (tx_flt[transmitmailbox] != DevFlt::kNone)
            return (tx_flt[transmitmailbox]);
    };

    return ( getFaultStatus(CANPERIPH_TX, transmitmailbox) );   // Return the current fault status
}

CANPeriph::DevFlt CANPeriph::poleAddTxRemoteMessage(uint32_t identifier, uint8_t size) {
/**************************************************************************************************
 * Populate the transmit mailbox. With a Data Frame request, will wait indefinitely until there is
 * space in the buffer to transmit the data.
 *************************************************************************************************/
    Form data_frame = { 0 };

    if ( (size > 8) || (size == 0) ) {      // If there is no data, or is greater then amount
                                            // allowed
        return (DevFlt::kData_Error);       // return error with DATA (also update fault state)
    }

    data_frame.Identifier   = identifier;
    data_frame.type         = RemTrans::kRemoteFrame;
    data_frame.size         = size;

    // Wait until at least one of the mailboxes is available
    while( ((_can_handle_->Instance->TSR & CAN_TSR_TME0) == 0U) &&
           ((_can_handle_->Instance->TSR & CAN_TSR_TME1) == 0U) &&
           ((_can_handle_->Instance->TSR & CAN_TSR_TME2) == 0U) )
    {}

    // Determine which mailbox is available to be populated
    uint8_t transmitmailbox = getFreeTxMailbox();

    // Check transmit mailbox value, if this ever exceeds 2 then just loop.
    //   Note, this shouldn't ever happen, this has been copied from the STM32 code - as protection
    while(transmitmailbox > 2) {
        transmitmailbox = getFreeTxMailbox();
    }

    fillTransmitMailBox(data_frame, transmitmailbox);
    requestTxMailboxTransmission(transmitmailbox);

    while(getTxRequestComplete(transmitmailbox) != 0) {
        getFaultStatus(CANPERIPH_TX, transmitmailbox);

        if (tx_flt[transmitmailbox] != DevFlt::kNone)
            return (tx_flt[transmitmailbox]);
    };

    return ( getFaultStatus(CANPERIPH_TX, transmitmailbox) );   // Return the current fault status
}

CANPeriph::Form CANPeriph::poleReadRxDataMessage(void) {
/**************************************************************************************************
 * Wait until there is any CAN message received (after filter) and provide as function output
 *************************************************************************************************/
    uint8_t     messagereceived = 0;
    uint8_t     FIFO_location   = 0;

    while (messagereceived == 0) {
        for (FIFO_location = 0; FIFO_location != 2; FIFO_location++) {
            messagereceived = getRxFIFOFillLevel(  FIFO_location  );

            if (messagereceived != 0) {  break;  }
        }
    }

    Form read_data = getRxFIFOMessage( FIFO_location );
    releaseRxFIFOMessage( FIFO_location );
    getFaultStatus(CANPERIPH_RX, FIFO_location);

    return ( read_data );        // Return the current fault status
}

void CANPeriph::configRxPendingIT(uint8_t RxFIFO, InterState intr) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable/disable the Rx Reception of a new message interrupt; where RxFIFO is the
 * FIFO for which the interrupt is enabled for
 *************************************************************************************************/
    uint32_t rx_FIFO_interrupt_register = 0;
    if (RxFIFO == 0)
        rx_FIFO_interrupt_register = CAN_IT_RX_FIFO0_MSG_PENDING;
    else
        rx_FIFO_interrupt_register = CAN_IT_RX_FIFO1_MSG_PENDING;


    if (intr == InterState::kIT_Enable) {                   // If request is to enable
        __HAL_CAN_ENABLE_IT(_can_handle_, rx_FIFO_interrupt_register);
    }
    else {                                                  // If request is to disable
        __HAL_CAN_ENABLE_IT(_can_handle_, rx_FIFO_interrupt_register);
    }
}

void CANPeriph::configRxOverRunIT(uint8_t RxFIFO, InterState intr) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable/disable the Rx Overrun condition interrupt; where RxFIFO is the
 * FIFO for which the interrupt is enabled for
 *************************************************************************************************/
    uint32_t rx_FIFO_interrupt_register = 0;
    if (RxFIFO == 0)
        rx_FIFO_interrupt_register = CAN_IT_RX_FIFO0_OVERRUN;
    else
        rx_FIFO_interrupt_register = CAN_IT_RX_FIFO1_OVERRUN;


    if (intr == InterState::kIT_Enable) {                   // If request is to enable
        __HAL_CAN_ENABLE_IT(_can_handle_, rx_FIFO_interrupt_register);
    }
    else {                                                  // If request is to disable
        __HAL_CAN_ENABLE_IT(_can_handle_, rx_FIFO_interrupt_register);
    }
}

void CANPeriph::configRxFullIT   (uint8_t RxFIFO, InterState intr) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable/disable the Rx full condition interrupt; where RxFIFO is the
 * FIFO for which the interrupt is enabled for
 *************************************************************************************************/
    uint32_t rx_FIFO_interrupt_register = 0;
    if (RxFIFO == 0)
        rx_FIFO_interrupt_register = CAN_IT_RX_FIFO0_FULL;
    else
        rx_FIFO_interrupt_register = CAN_IT_RX_FIFO1_FULL;

    if (intr == InterState::kIT_Enable) {                   // If request is to enable
        __HAL_CAN_ENABLE_IT(_can_handle_, rx_FIFO_interrupt_register);
    }
    else {                                                  // If request is to disable
        __HAL_CAN_ENABLE_IT(_can_handle_, rx_FIFO_interrupt_register);
    }
}

void CANPeriph::configTransmtIT  (InterState intr) {
/**************************************************************************************************
 * INTERRUPTS:
 * This will enable/disable the Transmit Buffer Empty interrupt.
 *************************************************************************************************/
    if (intr == InterState::kIT_Enable) {                   // If request is to enable
        __HAL_CAN_ENABLE_IT(_can_handle_, CAN_IT_TX_MAILBOX_EMPTY);
    }
    else {                                                  // If request is to disable
        __HAL_CAN_ENABLE_IT(_can_handle_, CAN_IT_TX_MAILBOX_EMPTY);
    }
}

void CANPeriph::intAddTxDataMessage(uint32_t identifier, uint8_t *wData, uint8_t size) {
/**************************************************************************************************
 * Populate the transmit mailbox. With a Data Frame request, interrupt version
 *************************************************************************************************/
    Form data_frame = { 0 };

    if ( (size > 8) || (size == 0) ) {      // If there is no data, or is greater then amount
                                            // allowed
        size = 8;                           // Force the value to 8
    }

    data_frame.Identifier   = identifier;
    data_frame.type         = RemTrans::kDataFrame;

    if ( (size > 8) || (size == 0) ) {      // If there is no data, or is greater then amount
                                            // allowed
        size = 8;                           // Force the value to 8
        // and populate the output with zeros - i.e. do not update from default setup
    }
    else {
        data_frame.size         = size;

        for (uint8_t i = 0; i != size; i++) {
            data_frame.data[i] = wData[i];
        }
    }

    // Put this data into the internal transmit buffer
    _form_wrte_q_.inputWrite(  data_frame  );
    startInterrupt();
}

_GenBufState CANPeriph::intGetRxMessage(CANPeriph::Form *read_message) {
/**************************************************************************************************
 * Check to see if there is any new messages in the class read buffer.
 * If there is, then update pointer contents to this - and return "kGenBuffer_New_Data"
 *          isn't, then return "kGenBuffer_Empty"
 *************************************************************************************************/
    if (_form_read_q_.state() != kGenBuffer_Empty) {
        _form_read_q_.outputRead(read_message);
        return ( kGenBuffer_New_Data );
    }

    return ( kGenBuffer_Empty );
}

void CANPeriph::startInterrupt(void) {
/**************************************************************************************************
 * Function will be called to start off a new CAN communication if there is something in the
 * queue, and the bus is free.
 *************************************************************************************************/
    startInterrupt( getFreeTxMailbox() );
}

void CANPeriph::startInterrupt(uint8_t mailbox_number) {
/**************************************************************************************************
 * OVERLOADED function, to only be called by interrupt.
 * Will populate the specific mailbox number, if there is something to be stored in the mailbox
 *************************************************************************************************/
    if ( (_form_wrte_q_.state() != kGenBuffer_Empty) ) {
        // At this point there is a new request to add data onto the CAN bus. However, before this
        // can be pushed onto the hardware, need to determine how many mailboxes are free...

        // If any of the hardware transmit mailboxes are available
        if (((_can_handle_->Instance->TSR & CAN_TSR_TME0) != 0U) ||
            ((_can_handle_->Instance->TSR & CAN_TSR_TME1) != 0U) ||
            ((_can_handle_->Instance->TSR & CAN_TSR_TME2) != 0U) ) {

            Form temp_form = { 0 };
            _form_wrte_q_.outputRead(&temp_form);
            fillTransmitMailBox(temp_form, mailbox_number);
            requestTxMailboxTransmission(mailbox_number);
        }
        else {
            // If there is no mailbox available for communication. Then need to wait for one to be
            // available...
        }
    }
    else if ( (_form_wrte_q_.state() == kGenBuffer_Empty) ) {
        //Disable();
    }
}

void CANPeriph::handleRxFIFOIRQ(uint8_t RxFIFO) {
/**************************************************************************************************
 * INTERRUPTS:
 * Interrupt Service Routine for the CAN events within the CAN class.
 *
 * The enabling of the interrupt for embedded devices (STM32) is done by enabling them via the
 * STM32cube GUI, and then in the main loop, enabling the required interrupts.
 * Then place this Interrupt routine handler within the function call for the interrupt vector.
 * For the Raspberry Pi, it involves creating a separate thread to handle the interrupts on a
 * periodic bases - essentially emulating the interrupt vector triggered from embedded devices.
 *
 * Function will go each of the status indicators which can trigger and interrupt, and see which
 * ones are enabled. If both a status event has occurred, and the interrupt is enabled, then this
 * function will take action.
 * Events covered by this function:
 *      Receive Overrun
 *          - If this occurs, then this will be recorded in a temporary variable within this
 *            function
 *
 *      Receive Pending or Receive Full
 *          - If there is any data within the 'RxFIFO' (either full, or 1) then the contents of
 *            the FIFO will be moved into the class internal buffer.
 *            The RxFIFO will then be released.
 *            The fault flag for the rx_flt will be updated for the 'RxFIFO'
 *
 *      No other interrupts are currently supported.
 *************************************************************************************************/
    DevFlt receive_fault = DevFlt::kNone;
    // If an Overrun event has been detected then
    if ( (receiveOverRunChk(RxFIFO) & receiveOverRunITChk(RxFIFO)) ) {
        // Capture an OVERRUN fault
        receive_fault = DevFlt::kOverRun;
    }

    // If there is anything in the receive buffer than, read it all and put into the class extended
    // buffer
    if (  (receiveFullChk(RxFIFO)           & receiveFullITChk(RxFIFO)) ||
         ((getRxFIFOFillLevel(RxFIFO) != 0) & receivePendingITChk(RxFIFO)) ) {
        uint8_t fill_level = getRxFIFOFillLevel(RxFIFO);

        while (fill_level != 0) {
            // Add the data to the internal buffer
            _form_read_q_.inputWrite( getRxFIFOMessage( RxFIFO ) );
            getFaultStatus(CANPERIPH_RX, RxFIFO);   // Get the status of the CAN BUS
            releaseRxFIFOMessage(RxFIFO);           // Ensure to release, so as to make the next
                                                    // data point available
            fill_level--;
        }

        // If there was a fault at the start of the interrupt, then force onto the class fault
        // state
        if (receive_fault != DevFlt::kNone)
            rx_flt[RxFIFO] = receive_fault;
    }
}

void CANPeriph::handleTxFIFOIRQ(void) {
/**************************************************************************************************
 * INTERRUPTS:
 * Interrupt Service Routine for the CAN events within the CAN class.
 *
 * The enabling of the interrupt for embedded devices (STM32) is done by enabling them via the
 * STM32cube GUI, and then in the main loop, enabling the required interrupts.
 * Then place this Interrupt routine handler within the function call for the interrupt vector.
 * For the Raspberry Pi, it involves creating a separate thread to handle the interrupts on a
 * periodic bases - essentially emulating the interrupt vector triggered from embedded devices.
 *
 * Function will go each of the status indicators which can trigger and interrupt, and see which
 * ones are enabled. If both a status event has occured, and the interrupt is enabled, then this
 * function will take action.
 * Events covered by this function:
 *      Transmit Mailbox empty/Transmission completed
 *          - Will determine which of the Tx Mailboxes has been completed, and is now available
 *            for populating with new data.
 *            Will clear the 'RQCP' flag; so as to clear the interrupt.
 *            Capture the status of the fault register and store in the tx_flt array
 *            Check to see if there are any other CAN requests within the class internal buffer,
 *            if so - populate into the now free Mailbox
 *
 *      No other interrupts are currently supported.
 *************************************************************************************************/
    uint8_t transmit_state = transmitEmptyChk();

    if ( ((transmit_state != 0) & transmitEmptyITChk()) ) {
        // If there is now mailboxes available for communication then...
        if ((transmit_state & 0x01) != 0U) {
            __HAL_CAN_CLEAR_FLAG(_can_handle_, CAN_FLAG_RQCP0);
            // Further fault status information can be retrieved here is required
            getFaultStatus(CANPERIPH_TX, 0);

            startInterrupt(0);
        }

        if ((transmit_state & 0x02) != 0U) {
            __HAL_CAN_CLEAR_FLAG(_can_handle_, CAN_FLAG_RQCP1);
            // Further fault status information can be retrieved here is required
            getFaultStatus(CANPERIPH_TX, 1);

            startInterrupt(1);
        }

        if ((transmit_state & 0x04) != 0U) {
            __HAL_CAN_CLEAR_FLAG(_can_handle_, CAN_FLAG_RQCP2);
            // Further fault status information can be retrieved here is required
            getFaultStatus(CANPERIPH_TX, 2);

            startInterrupt(2);
        }

        // There is no need to clear the interrupt enabling bit; as the trigger for this is the
        // communication request being completed
    }
}

//void CANPeriph:: handleErrorIRQ(void) {
/**************************************************************************************************
 * INTERRUPTS:
 * Interrupt Service Routine for the CAN errors within the CAN class.
 *
 * The enabling of the interrupt for embedded devices (STM32) is done by enabling them via the
 * STM32cube GUI, and then in the main loop, enabling the required interrupts.
 * Then place this Interrupt routine handler within the function call for the interrupt vector.
 * For the Raspberry Pi, it involves creating a separate thread to handle the interrupts on a
 * periodic bases - essentially emulating the interrupt vector triggered from embedded devices.
 *
 * Function will go each of the status indicators which can trigger and interrupt, and see which
 * ones are enabled. If both a status event has occurred, and the interrupt is enabled, then this
 * function will take action.
 * Events covered by this function:

 *
 *      No other interrupts are currently supported.
 *************************************************************************************************/
    // Not supported currently.
    // The transmit and receive interrupts already read the fault registers
//}

CANPeriph::~CANPeriph()
{
    // TODO Auto-generated destructor stub
}

