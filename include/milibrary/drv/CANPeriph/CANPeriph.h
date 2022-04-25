/**************************************************************************************************
 * @file        CANPeriph.h
 * @author      Thomas
 * @brief       Header file for the Generic CANPeriph Class handle
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * 
 * 
 *************************************************************************************************/
#ifndef CANPeriph_H_
#define CANPeriph_H_

#include "FileIndex.h"
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

// Defines specific within this class
// None
// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/

class CANPeriph {
public:
    // Constants used within this class, that are used externally.
    static const uint32_t   kExtendedFrame  = 0x80000000;
    static const uint32_t   kStandardFrame  = 0x00000000;

/**************************************************************************************************
 * ==   TYPES   == >>>       TYPES GENERATED WITHIN CLASS        <<<
 *   -----------
 *  Following types are generated within this class. If needed outside of the class, need to
 *  state "CANPeriph::" followed by the type.
 *************************************************************************************************/
public:
     enum class DevFlt : uint8_t {   // Fault Type of the class (internal enumerate)
         kNone           = 0x00,     // Normal Operation
         // Structure of the next 6 faults align with those captured in the STM32 'Last Error Code'
         kStuff_Error    = 0x01,     // Stuff Error
         kForm_Error     = 0x02,     // Form Error
         kAck_Error      = 0x03,     // Acknowledgement Error
         kBit_Recessive  = 0x04,     // Bit Recessive Error
         kBit_Dominant   = 0x05,     // Bit Dominant Error
         kCRC_Error      = 0x06,     // CRC Error

         kData_Error     = 0x11,     // Data Error

         kOverRun        = 0x20,     // Indicate a OverRun fault (specific for the RxFIFO's)

         kTime_Out       = 0xFE,     // Timeout of function(s)
         kInitialised    = 0xFF      // Just initialised
     };

     enum InterState : uint8_t  {kIT_Enable, kIT_Disable};       // Enumerate state for enabling/
                                                                 // disabling interrupts
     enum RemTrans :uint8_t     {kRemoteFrame, kDataFrame};      // Enumerate state for configuring
                                                                 // of frame for message


     enum FiltScale : uint8_t   {k16bit, k32bit};                // Enumerate for the filter scaler
                                                                 // mode
     enum FiltMode  : uint8_t   {KID_Mask, kID_List};            // Enumerate for the filter mode
     enum FiltEnb   : uint8_t   {kFilt_Enable, kFilt_Disable};   // Enumerate for filter enabling


     typedef struct {           // CAN Form structure, used to manage CAN Communication
                                // interrupts
         uint8_t                size;       // State the amount of data to be transferred

         uint32_t               Identifier; // Identifier for the message (Standard/Extended)
         /*
          * If bit[31] = "0"
          *     Identifier is 'Standard' - with bits [10:0] being the identifier
          *
          * if bit[31] = "1"
          *     Identifier is 'Extended' - with bits [28:18] Upper identifier
          *                                with bits  [17:0] Lower identifier
          */
         uint8_t                data[8];    // Register to contain the data to be transmitted

         RemTrans               type;       // Message frame type
     }   Form;

 /**************************************************************************************************
 * == GEN PARAM == >>>       GENERIC PARAMETERS FOR CLASS        <<<
 *   -----------
 *  Parameters required for the class to function.
 *************************************************************************************************/
    protected:
        GenBuffer<Form>     _form_wrte_q_;  // Pointer to the class internal UARTForm buffer, which
        GenBuffer<Form>     _form_read_q_;  // is used to manage interrupt based communication.
                                            // Functions will add request forms to this buffer,
                                            // and interrupt then goes through them sequentially.
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    public:
        DevFlt      tx_flt[3];          // Fault status of the transmit buffers
        DevFlt      rx_flt[2];          // Fault status of the receive buffers
        uint8_t     wrte_error_count;
        uint8_t     read_error_count;

/**************************************************************************************************
 * == SPC PARAM == >>>        SPECIFIC ENTRIES FOR CLASS         <<<
 *   -----------
 *  Following are functions and parameters which are specific for the embedded device selected.
 *  The initialisation function for the class is also within this section, which again will be
 *  different depending upon the embedded device selected.
 *************************************************************************************************/
    protected:
        void popGenParam(void);         // Populate generic parameters for the class

#if   ( (zz__MiEmbedType__zz == 50) || (zz__MiEmbedType__zz == 51)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================
    protected:
        CAN_HandleTypeDef   *_can_handle_;      // Store the CAN handle

    public:
        CANPeriph(CAN_HandleTypeDef *CAN_Handle, Form *WrteForm, uint16_t WrteFormSize,
                  Form *ReadForm, uint16_t ReadFormSize);
        // Setup the CAN class, for STM32 by providing the CAN Request Form array pointer, as
        // well as the size.
        // Class will then generate a GenBuffer item internally.

#else
    public:
        CANPeriph();

#endif

    virtual ~CANPeriph();

/**************************************************************************************************
 * == GEN FUNCT == >>>      GENERIC FUNCTIONS WITHIN CLASS       <<<
 *   -----------
 *  The following are functions scoped within the "CANPeriph" class, which are generic; this means
 *  are used by ANY of the embedded devices supported by this class.
 *  The internals of the class, will then determine how it will be managed between the multiple
 *  embedded devices.
 *************************************************************************************************/
protected:  /**************************************************************************************
             * == PROTECTED == >>>     DIRECT HARDWARE READING FUNCTIONS     <<<
             *   -----------
             *  Functions allow direct access to the hardware/base functions of this class. Are
             *  not visible unless "friend" or "child"/inherited
             *************************************************************************************/
    uint8_t     getRxFIFOFillLevel(uint8_t RxFIFO);             // Retrieve the level of FIFO (Rx)
    uint8_t     getTxMailboxesFreeLevel(void);                  // Retrieve the level of FIFO (Tx)
    DevFlt      getFaultStatus(uint8_t is_read, uint8_t FIFO);  // Read the status of the fault
                                                                // register(s), and populate into
                                                                // class internals

    uint8_t     getTxRequestComplete(uint8_t mailbox_number);   // Check state of transmission
                                                                // (1 = Complete)
    uint8_t     getFreeTxMailbox(void);                         // Retrieve the next available
                                                                // Tx Mailbox
    void        releaseRxFIFOMessage(uint8_t RxFIFO);
        // Release the selected Rx FIFO so can be populated with new messages
    void        requestTxMailboxTransmission(uint8_t mailbox_number);
        // Set hardware to transmit selected mailbox when able to

    void        fillTransmitMailBox(Form can_message, uint8_t mailbox_number);
    Form        getRxFIFOMessage(uint8_t RxFIFO);
        // Functions to populate the hardware with the requested contents of 'can_message' or
        // read the hardware contents and return as function output

    void        configFilter(uint8_t filter_bank, FiltMode mode, FiltScale scale, uint8_t RxFIFO,
                             FiltEnb filter_enable, uint32_t fr1, uint32_t fr2);
        // Configure the selected CAN filter 'filter_bank'

    // CAN Event status checks
    // ~~~~~~~~~~~~~~~~~~~~~~~
    uint8_t receiveOverRunChk(uint8_t RxFIFO);      // Check state of Rx Overrun (1 = Overrun)
    uint8_t receiveFullChk   (uint8_t RxFIFO);      // Check state of Rx full    (1 = Full)
    uint8_t transmitEmptyChk(void);                 // Check state of Transmit empty (1 = empty)

    // CAN Interrupt status checks
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~
    uint8_t receivePendingITChk(uint8_t RxFIFO);    // Check to see if interrupt is enabled
    uint8_t receiveOverRunITChk(uint8_t RxFIFO);    //     (1 = enabled)
    uint8_t receiveFullITChk   (uint8_t RxFIFO);    //
    uint8_t transmitEmptyITChk(void);               //

public:     /**************************************************************************************
             * ==  PUBLIC   == >>>   CONFIGURATE FUNCTIONS FOR THE HARDWARE  <<<
             *   -----------
             *  Visible functions used to configure the hardware of the CAN - mainly on the filters
             *************************************************************************************/
    void        config32bitFilter(uint8_t filter_bank, FiltMode mode, uint8_t RxFIFO,
                                  FiltEnb filter_enable,
                                  uint32_t id, uint32_t mask);
    void        config16bitFilter(uint8_t filter_bank, FiltMode mode, uint8_t RxFIFO,
                                  FiltEnb filter_enable,
                                  uint16_t mask_id_high,  uint16_t id_high,
                                  uint16_t mask_id_low,   uint16_t id_low);
    /* Simplified versions of 'configFilter', so as to allow for easy reading of what filter
     * configuration has been used.
     * 'config32bitFilter' will configure the hardware for a 2x32bit filter
     * 'config16bitFilter' will configure the hardware for a 4x16bit filter
     */

    static uint16_t    filterID16bit  (uint32_t identifier);
    static uint16_t    filterMASK16bit(uint32_t identifier);
    static uint32_t    filterID32bit  (uint32_t identifier);
    static uint32_t    filterMASK32bit(uint32_t identifier);
    /* Ease of use functions, will provide as output the requested format compatible with the
     * STM32 filter hardware.
     * NOTE - 'MASK' versions are similar to the 'ID' versions. However, will enforce Extended ID
     *        and Remote Message to match the specified 'ID'
     *
     * Examples:
     *  1. Create a 32bit filter, to capture Standard IDs 0 to 7, and store any matches in
     *     RxFIFO[0].
     *       Note - this will be the highest priority filter as 'filter_bank' is zero
     *      config32bitFilter(0, FiltMode::KID_Mask, 0, FiltEnb::kFilt_Enable,
     *                        filterID32bit  (0x000)    -- ID
     *                        filterMASK32bit(0x7F8));  -- MASK
     *
     *  2. Create a 16bit filter, capture 4 specific Extended IDs, and store any matches in
     *     RxFIFO[1]
     *       Note - this will be the lowest priority filter as 'filter_bank' is 13
     *      config16bitFilter(13,  FiltMode::kID_List, 1, FiltEnb::kFilt_Enable,
     *                        filterID16bit  (kExtendedFrame | (0x008 << 18) | (0x18000)),
     *                        filterID16bit  (kExtendedFrame | (0x088 << 18) | (0x18000)),
     *                        filterID16bit  (kExtendedFrame | (0x488 << 18) | (0x18000)),
     *                        filterID16bit  (kExtendedFrame | (0x001 << 18) | (0x38000)));
     */


public:     /**************************************************************************************
             * ==  PUBLIC   == >>>    POLING FUNCTIONS FOR DATA TRANSFER     <<<
             *   -----------
             *  Visible functions used to transfer data via CAN - will wait for any registers to
             *  be in correct state before progressing.
             *************************************************************************************/
    void    abortTxMailbox(uint8_t mailbox_bitpack_number);     // Abort the requested Mailbox
                                                                // transmission

    DevFlt  poleAddTxDataMessage  (uint32_t identifier, uint8_t *wData, uint8_t size);
    DevFlt  poleAddTxRemoteMessage(uint32_t identifier, uint8_t size);
    Form    poleReadRxDataMessage(void);

public:     /**************************************************************************************
             * ==  PUBLIC   == >>>   INTERRUPT FUNCTIONS FOR DATA TRANSFER   <<<
             *
             *  Visible functions used to transfer data via UART - in Interrupt mode, so allows
             *  other functions to run, whilst hardware "does its thing!"
             *************************************************************************************/
    void configRxPendingIT(uint8_t RxFIFO, InterState intr);    // Configure the Pending Receive
    void configRxOverRunIT(uint8_t RxFIFO, InterState intr);    // Configure the Receive Overrun
    void configRxFullIT   (uint8_t RxFIFO, InterState intr);    // Configure the Receive Full
    void configTransmtIT  (InterState intr);                    // Configure the Transmit Empty

    void intAddTxDataMessage(uint32_t identifier, uint8_t *wData, uint8_t size);
    _GenBufState intGetRxMessage(Form *read_message);

    void startInterrupt(void);                      // Enable communication if bus is free,
    void startInterrupt(uint8_t mailbox_number);    // otherwise wait (doesn't actually wait)

    void handleRxFIFOIRQ(uint8_t RxFIFO);           // Interrupt Handle for CAN Device
    void handleTxFIFOIRQ(void);                     //
    //void handleErrorIRQ(void);                      //
};

#endif /* CANPeriph_H_ */
