/**************************************************************************************************
 * @file        UARTPeriph.h
 * @author      Thomas
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
 *      Call Class UARTPeriph to initialise the class
 *          For STM32F devices, providing the address of the UART handler - from cubeMX
 *          For STM32L devices, providing the address of the UART handler - from cubeMX
 *          For RaspberryPi, provide the location of the serial interface, and the desired baudrate
 *          #### UPDATE TO THE FORM SYSTEM HAS NOT BEEN TESTED WITHIN RASPBERRY PI YET!
 *
 *
 *          Additional to this the size of the UART buffer array is required.
 *
 *      Depending upon how the programmer wants to use the UART device, will change which functions
 *      are utilised.
 *      For functions to wait for new data, or data to be transmitted use:
 *          ".poleSingleRead"       - Wait for new data to be read via UART
 *          ".poleSingleTransmit"   - Wait for data to be transmitted via UART
 *          ".poleTransmit"         - As above, however can transmit multiple data (array)
 *
 *      If interrupts are to be used, then will first need to be configured within the NVIC (which
 *      is not part of this class), then the following can be used:
 *          ".configTransmtIT"      - Enable/Disable Transmit Empty interrupt
 *          ".configTransCmIT"      - Enable/Disable Transmission complete interrupt
 *          ".configReceiveIT"      - Enable/Disable Receive buffer full interrupt
 *
 *          ".intWrtePacket"        - Puts a request for a write of data (out of STM) via UART,
 *                                    (utilises the UART form system, see below), expects to
 *                                    receive an array data location
 *
 *          ".intReadPacket"        - Puts a request to read back data (in to STM) via UART,
 *                                    (again utilises the UART form system, see below), expects to
 *                                    receive an array data location
 *
 *          ".startInterrupt"       - Check to see if the UART bus is free, and a new request form
 *                                    (either read or write) is available. Then trigger a
 *                                    communication run (Enables Transmit Empty/Receive interrupts)
 *          ".intWrteFormCmplt"     - Function will go through tidy up procedure for the current
 *                                    Request form - Write buffer
 *          ".intReadFormCmplt"     - Same as above however for the Read buffer
 *          ".handleIRQ"            - Functions to be placed within the relevant Interrupt Vector
 *                                    call, so as to handle the UART interrupt
 *
 *          ".readGenBufferLock"    - Specific function for the read buffer, will essential lock
 *                                    the read buffer to a single array (based upon input
 *                                    GenBuffer), ensure that pointer aligns with any new reads,
 *                                    and if the buffer has been filled (i.e. will therefore no
 *                                    longer reading data back into the array), will put a new
 *                                    read request back into the system.
 *                                    (Intended for Interrupt based communication only - DMA
 *                                     will have own specific version)
 *
 *      Following functions are protected, so will only work for classes which inherit from this
 *      one, and not visible external to class:
 *          ".readDR"               - Will take data straight from the hardware
 *          ".writeDR"              - Will put data straight onto the hardware
 *
 *          ".transmitEmptyChk"     - Check to see if the Transmit Empty buffer is empty
 *          ".transmitComptChk"     - Check to see if Transmission is complete
 *          ".receiveToReadChk"     - Check to see if the Receive buffer is full (data to read)
 *
 *          ".transmitEmptyITChk"   - Indicates if the interrupt for Transmit Empty has been
 *                                    enabled
 *          ".transmitComptITChk"   - Indicates if the interrupt for Transmit Complete has been
 *                                    enabled
 *          ".receiveToReadITChk"   - Indicates if the interrupt for Receive buffer full has been
 *                                    enabled
 *
 *  [#] UART Request Form System
 *      ~~~~~~~~~~~~~~~~~~~~~~~~
 *      Whilst utilising minimised CPU loaded communication, the UART Device will utilise a UART
 *      Request Form system, so as to ensure that each of the multiple source functions wanting to
 *      communication via UART, is done correctly and effectively.
 *      Each of the source functions will generate a Request Form, which will then be placed into
 *      the target UART device Form queue (for reading and writing), such that the UART device can
 *      go through each of these forms in sequence; each form will contain the following:
 *          Number of packets to transmit/write (8bits x N)
 *          Location for where the data is to be taken/stored
 *          Communication complete return flag      (will be updated with the amount of packets
 *                                                   transmitted successfully)
 *          UART communication fault return flag
 *
  *      Function list (all are protected):
 *          ".genericForm"          - Populate generic entries of the UART Form (outputs structure)
 *          ".formW8bitArray"       - Link form to a 8bit array location
 *
 *          ".getFormWriteData"     - Retrieve data from UART Form's requested location
 *          ".putFormReadData"      - Write data to location specified by current UART Form
 *
 *      There is no other functionality within this class.
 *************************************************************************************************/
#ifndef UARTPeriph_H_
#define UARTPeriph_H_

#include "FileIndex.h"
// C System Header(s)
// ------------------
#include <stdint.h>

// C++ System Header(s)
// --------------------

// Other Libraries
// --------------
#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
#include "stm32f1xx_hal.h"              // Include the HAL UART library

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
#include "stm32l4xx_hal.h"              // Include the HAL UART library

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================
#include <wiringSerial.h>               // Include the wiringPi UART/Serial library

#else
//=================================================================================================
#error "Unrecognised target device"

#endif

// Project Libraries
// -----------------
#include FilInd_GENBUF_TP               // Provide the template for the circular buffer class

//=================================================================================================


class UARTPeriph {
/**************************************************************************************************
* ==   TYPES   == >>>       TYPES GENERATED WITHIN CLASS        <<<
*   -----------
*  Following types are generated within this class. If needed outside of the class, need to
*  state "UARTPeriph::" followed by the type.
**************************************************************************************************/
public:
     enum class DevFlt : uint8_t {   // Fault Type of the class (internal enumerate)
         kNone           = 0x00,     // Normal Operation
         kData_Error     = 0x01,     // Data Error
         kParity         = 0x02,     // Parity Fault

         kDMA_Rx_Error   = 0xFD,     // Error triggered if DMA (Receive) error
         kDMA_Tx_Error   = 0xFE,     // Error triggered if DMA (Transmit) errorST
         kInitialised    = 0xFF      // Just initialised
     };

     enum InterState : uint8_t {kIT_Enable, kIT_Disable};   // Enumerate state for enabling/
                                                            // disabling interrupts
     enum CommLock : uint8_t {kCommunicating, kFree};       // Enumerate state for indicating if
                                                            // device iscommunicating

     typedef struct {           // UART Form structure, used to manage UART Communication
                                // interrupts
         uint16_t               size;       // State the amount of data to be transferred

         uint8_t                *Buff;      // Pointer to array to contain data

         volatile uint16_t       *Cmplt;     // Provide a pointer to a "Complete" flag (will be
                                             // incremented) - to be cleared by source function
         volatile DevFlt         *Flt;       // Provide a pointer to a UARTPeriph::DevFlt for the
                                             // I2C fault status to be provided to source
                                             // function
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
        Form            _cur_wrte_form_;    // Current UART Read form
        uint16_t        _cur_wrte_count_;   // Current communication packet count (Write)

        Form            _cur_read_form_;    // Current UART Read form
        uint16_t        _cur_read_count_;   // Current communication packet count (Read)

    public:
        DevFlt      flt;                // Fault state of the UART Device
        CommLock    wrte_comm_state;    // Status of the Communication (Write)
        CommLock    read_comm_state;    // Status of the Communication (Read)

/**************************************************************************************************
 * == SPC PARAM == >>>        SPECIFIC ENTRIES FOR CLASS         <<<
 *   -----------
 *  Following are functions and parameters which are specific for the embedded device selected.
 *  The initialisation function for the class is also within this section, which again will be
 *  different depending upon the embedded device selected.
 *************************************************************************************************/
    protected:
        void popGenParam(void);         // Populate generic parameters for the class

#if ( defined(zz__MiSTM32Fx__zz) || defined(zz__MiSTM32Lx__zz)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//==================================================================================================
    protected:
        UART_HandleTypeDef  *_uart_handle_;     // Store the UART handle

    public:
        UARTPeriph(UART_HandleTypeDef *UART_Handle, Form *WrteForm, uint16_t WrteFormSize,
                   Form *ReadForm, uint16_t ReadFormSize);
        // Setup the UART class, for STM32 by providing the UART Request Form array pointer, as
        // well as the size.
        // Class will then generate a GenBuffer item internally.

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    private:
        int                 _uart_handle_;      // Stores the device to communicate too
        const char          *_device_loc_;      // Store location file for UART device
        int                 _baud_rate_;        // Store entered baudrate
        uint8_t             _pseudo_interrupt_; // Pseudo interrupt register

    public:
        int  anySerDataAvil(void);              // Function to provide the amount of data at
                                                // hardware baundry

        UARTPeriph(const char *deviceloc, int baud, Form *WrteForm, uint16_t WrteFormSize,
                                                    Form *ReadForm, uint16_t ReadFormSize);
        // Setup the UART class, by providing the folder location of serial interface, and baudrate
        // as well the "GenBuffer" needing to be provided to the function, to be fully defined
        // outside of class

#else
//==================================================================================================
    public:
        UARTPeriph();

#endif

/**************************************************************************************************
 * == GEN FUNCT == >>>      GENERIC FUNCTIONS WITHIN CLASS       <<<
 *   -----------
 *  The following are functions scoped within the "UARTPeriph" class, which are generic; this means
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
    //void Enable(void);                      // Enable the UART device
    //void Disable(void);                     // Disable the UART device

    uint8_t readDR(void);                   // Function to read direct from the hardware
    void writeDR(uint8_t data);             // Function to write direct to the hardware

    // UART Event status checks
    // ~~~~~~~~~~~~~~~~~~~~~~~~~
    uint8_t transmitEmptyChk(void);         // Check state of transmission register (1 = empty)
    uint8_t transmitComptChk(void);         // Check state of transmission complete (1 = cmplt)
    uint8_t receiveToReadChk(void);         // Check state of receive data register (1 =  read)

    // UART Error status checks
    // ~~~~~~~~~~~~~~~~~~~~~~~~
    void clearComptChk(void);               // Clear the Transmission Complete flag

    // UART Interrupt status checks
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    uint8_t transmitEmptyITChk(void);       // Check to see if interrupt is enabled (1 = enabled)
    uint8_t transmitComptITChk(void);       // Check to see if interrupt is enabled (1 = enabled)
    uint8_t receiveToReadITChk(void);       // Check to see if interrupt is enabled (1 = enabled)

    // UART Communication Request Form handling
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Form genericForm(uint8_t *data, uint16_t size,
                     volatile DevFlt *fltReturn, volatile uint16_t *cmpFlag);

    uint8_t getFormWriteData(Form *RequestForm);
    // Function will retrieve the next data entry from the source data specified within the
    // UART "RequestForm"

    void putFormReadData(Form *RequestForm, uint8_t readdata);
    // Function will take the data read from the UART Hardware, and put into the requested data
    // location specified within the "RequestForm" input (will be curForm)

public:     /**************************************************************************************
             * ==  PUBLIC   == >>>    POLING FUNCTIONS FOR DATA TRANSFER     <<<
             *   -----------
             *  Visible functions used to transfer data via UART - will wait for any registers to
             *  be in correct state before progressing.
             *************************************************************************************/
    uint8_t poleSingleRead(void);               // Will wait until there is new data to be read
    void    poleSingleTransmit(uint8_t data);   // Will wait until it can transfer data via UART
    DevFlt  poleTransmit(uint8_t *pData, uint16_t size);    // Will wait until it can transfer
                                                            // data via UART, multiple entries

public:     /**************************************************************************************
             * ==  PUBLIC   == >>>   INTERRUPT FUNCTIONS FOR DATA TRANSFER   <<<
             *
             *  Visible functions used to transfer data via UART - in Interrupt mode, so allows
             *  other functions to run, whilst hardware "does its thing!"
             *************************************************************************************/
    void configTransmtIT(InterState intr);      // Configure the Transmit Empty interrupt
    void configTransCmIT(InterState intr);      // Configure the Transmit Complete interrupt
    void configReceiveIT(InterState intr);      // Configure the Receive full interrupt

    void intWrtePacket(uint8_t *wData, uint16_t size,
                       volatile DevFlt *fltReturn, volatile uint16_t *cmpFlag);

    void intReadPacket(uint8_t *rData, uint16_t size,
                       volatile DevFlt *fltReturn, volatile uint16_t *cmpFlag);

    virtual void startInterrupt(void);          // Enable communication is bus is free, otherwise
                                                // wait (doesn't actually pause at this point)
    virtual void intWrteFormCmplt(void);        // Closes out the input Request Form (Write)
    virtual void intReadFormCmplt(void);        // Closes out the input Request Form (Read)

    virtual void readGenBufferLock(GenBuffer<uint8_t> *ReadArray,
                                   volatile DevFlt *fltReturn, volatile uint16_t *cmpFlag);

    virtual void handleIRQ(void);               // Interrupt handler

        virtual ~UARTPeriph();
};

#endif /* UART_UART_H_ */
