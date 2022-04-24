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
         // Structure of the next 6 faults align with those captured in the STM32 'Last Erro Code'
         kStuff_Error    = 0x01,     // Stuff Error
         kForm_Error     = 0x02,     // Form Error
         kAck_Error      = 0x03,     // Acknowledgement Error
         kBit_Recessive  = 0x04,     // Bit Recessive Error
         kBit_Dominant   = 0x05,     // Bit Dominant Error
         kCRC_Error      = 0x06,     // CRC Error

         kData_Error     = 0x11,     // Data Error

         kOverRun        = 0x20,     // Indicate a OverRun fault (specific for the RxFifo's)

         kTime_Out       = 0xFE,     // Timeout of function(s)
         kInitialised    = 0xFF      // Just initialised
     };

     enum InterState : uint8_t {kIT_Enable, kIT_Disable};   // Enumerate state for enabling/
                                                            // disabling interrupts
     enum RemTrans :uint8_t {kRemoteFrame, kDataFrame};     // Enumerate state for configuring
                                                            // of frame for message


     enum FiltScale : uint8_t {k16bit, k32bit};             // Enumerate for the filter scaler mode
     enum FiltMode  : uint8_t {KID_Mask, kID_List};         // Enumerate for the filter mode
     enum FiltEnb   : uint8_t {kFilt_Enable, kFilt_Disable};// Enumerate for filter enabling


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
    public:
        GenBuffer<Form>     _form_wrte_q_;  // Pointer to the class internal UARTForm buffer, which
        GenBuffer<Form>     _form_read_q_;  // is used to manage interrupt based communication.
                                            // Functions will add request forms to this buffer,
                                            // and interrupt then goes through them sequentially.
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    public:
        DevFlt      tx_flt[3];          // Fault status of the transmit buffers
        DevFlt      rx_flt[2];          // Fualt status of the receive buffers
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
    uint32_t    getRxFifoFillLevel(uint8_t RxFifo);
    DevFlt      getFaultStatus(uint8_t is_read, uint8_t fifo);

    void        fillTransmitMailBox(Form can_message, uint8_t mailbox_number);
    uint8_t     getTxMailboxesFreeLevel(void);
    uint8_t     getTxRequestComplete(uint8_t mailbox_number);
    uint8_t     getFreeTxMailbox(void);
    void        requestTxMailboxTransmission(uint8_t mailbox_number);

    Form        getRxFifoMessage(uint8_t RxFifo);
    void        releaseRxFifoMessage(uint8_t RxFifo);
    void        configFilter(uint8_t filter_bank, FiltMode mode, FiltScale scale, uint8_t RxFifo,
                             FiltEnb filter_enable, uint32_t fr1, uint32_t fr2);

    // CAN Event status checks
    // ~~~~~~~~~~~~~~~~~~~~~~~
    uint8_t receiveOverRunChk(uint8_t RxFifo);      // Check state of Rx Overrun (1 = Overrun)
    uint8_t receiveFullChk   (uint8_t RxFifo);      // Check state of Rx full    (1 = Full)
    uint8_t transmitEmptyChk(void);                 // Check state of Transmit empty (1 = empty)

    // CAN Interrupt status checks
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~
    uint8_t receivePendingITChk(uint8_t RxFifo);    // Check to see if interrupt is enabled
    uint8_t receiveOverRunITChk(uint8_t RxFifo);    //     (1 = enabled)
    uint8_t receiveFullITChk   (uint8_t RxFifo);    //
    uint8_t transmitEmptyITChk(void);               //

public:     /**************************************************************************************
             * ==  PUBLIC   == >>>   CONFIGURATE FUNCTIONS FOR THE HARDWARE  <<<
             *   -----------
             *  Visible functions used to configure the hardware of the CAN - mainly on the filters
             *************************************************************************************/
    void        config32bitFilter(uint8_t filter_bank, FiltMode mode, uint8_t RxFifo,
                                  FiltEnb filter_enable,
                                  uint32_t id, uint32_t mask);

    void        config16bitFilter(uint8_t filter_bank, FiltMode mode, uint8_t RxFifo,
                                  FiltEnb filter_enable,
                                  uint16_t mask_id_high,  uint16_t id_high,
                                  uint16_t mask_id_low,   uint16_t id_low);

    static uint16_t    filterID16bit  (uint32_t identifier);
    static uint16_t    filterMASK16bit(uint32_t identifier);
    static uint32_t    filterID32bit  (uint32_t identifier);
    static uint32_t    filterMASK32bit(uint32_t identifier);

public:     /**************************************************************************************
             * ==  PUBLIC   == >>>    POLING FUNCTIONS FOR DATA TRANSFER     <<<
             *   -----------
             *  Visible functions used to transfer data via CAN - will wait for any registers to
             *  be in correct state before progressing.
             *************************************************************************************/
    void    abortTxMailbox(uint8_t mailbox_bitpack_number);

    DevFlt  poleAddTxDataMessage  (uint32_t identifier, uint8_t *wData, uint8_t size);
    DevFlt  poleAddTxRemoteMessage(uint32_t identifier, uint8_t size);
    Form    poleReadRxDataMessage(void);

public:     /**************************************************************************************
             * ==  PUBLIC   == >>>   INTERRUPT FUNCTIONS FOR DATA TRANSFER   <<<
             *
             *  Visible functions used to transfer data via UART - in Interrupt mode, so allows
             *  other functions to run, whilst hardware "does its thing!"
             *************************************************************************************/
    void configRxPendingIT(uint8_t RxFifo, InterState intr);    // Configure the Pending Receive
    void configRxOverRunIT(uint8_t RxFifo, InterState intr);    // Configure the Receive Overrun
    void configRxFullIT   (uint8_t RxFifo, InterState intr);    // Configure the Receive Full
    void configTransmtIT  (InterState intr);                    // Configure the Transmit Empty

    void intAddTxDataMessage(uint32_t identifier, uint8_t *wData, uint8_t size);
    void startInterrupt(void);                      // Enable communication if bus is free,
    void startInterrupt(uint8_t mailbox_number);    // otherwise wait (doesn't actually wait)

    void handleRxFIFOIRQ(uint8_t RxFifo);
    void handleTxFIFOIRQ(void);
    void handleErrorIRQ(void);
};

#endif /* CANPeriph_H_ */
