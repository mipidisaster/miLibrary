/**************************************************************************************************
 * @file        I2CPeriph.h
 * @author      Thomas
 * @version     V1.2
 * @date        08 Nov 2018
 * @brief       Header file for the Generic I2C Class handle
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * 
 * Class supports multiple target devices, depending upon which one is defined will modify how the
 * I2CPeriph class can be initialised.
 * The basic use of the class is the same for all target devices
 *      Call Class I2CPeriph to initialise the class
 *          For STM32L devices, providing the address of the I2C handler - from cubeMX
 *
 *      Depending upon how the programmer wants to use the I2C device, will change which functions
 *      are utilised.
 *      For functions to wait for new data, or data to be transmitted use (polling mode):
 *          ".poleMasterTransmit"   - Transmit data (in MASTER mode), waiting for states to be
 *                                    ready
 *          ".poleMasterReceive"    - Receive data (in MASTER mode), waiting for states to be ready
 *          ".poleDeviceRdy"        - Determine if the target I2C device is available to
 *                                    communicate
 *
 *      If interrupts are to be used, then will first need to be configured within the NVIC (which
 *      is not part of this class), then the following can be used:
 *          ".configTransmtIT"      - Enable/Disable Transmit Empty interrupt
 *          ".configTransCmIT"      - Enable/Disable Transmission complete interrupt
 *          ".configReceiveIT"      - Enable/Disable Receive buffer full interrupt
 *          ".configBusNACKIT"      - Enable/Disable I2C Bus NACK interrupt
 *          ".configBusSTOPIT"      - Enable/Disable I2C Bus STOP interrupt
 *          ".configBusErroIT"      - Enable/Disable I2C Bus Error interrupt
 *
 *          ".intMasterReq"         - Put a request for an interrupt based communication on the
 *                                    selected I2C device (utilises the I2C form system, see below)
 *                                    expects to receive a GenBuffer data location
 *
 *          ".I2CInterruptStart"    - Check to see if the I2C bus is free, and a new request form
 *                                    is available. Then trigger a communication run (enables
 *                                    Transmit Empty/Receive interrupts)
 *          ".intReqFormCmplt"      - Function will go through tidy up procedure for the current
 *                                    Request Form
 *          ".IRQEventHandle"       - Functions to be placed within the relevant Interrupt Vector
 *                                    call, so as to handle the I2C interrupt
 *          ".IRQErrorHandle"       - Functions to be placed within the relevant Interrupt Vector
 *                                    call, so as to handle the I2C interrupt
 *
 *      Following functions are protected so will only work for classes which inherit from this
 *      one, and are not visible external to this class. They contain the lower level handling
 *      of hardware, which the functions above rely upon to function (this is where a majority of
 *      the differences between the supported embedded devices will lie:
 *          ".DRRead"               - Will take data straight from hardware
 *          ".DRWrite"              - Will put data straight onto the hardware
 *          ".RequestTransfer"      - Configures the hardware for a I2C transfer (sets up Address,
 *                                    R/~W, START/STOP, etc.)
 *
 *          ".TransmitEmptyChk"     - Check to see if the Transmit Empty buffer is empty
 *          ".TransmitComptChk"     - Check to see if Transmission is complete
 *          ".ReceiveToReadChk"     - Check to see if the Receive buffer is full (data to read)
 *          ".BusNACKChk"           - Check to see if there has been a NACK on the BUS
 *          ".BusBusyChk"           - Check to see if the BUS is busy
 *          ".BusSTOPChk"           - Check to see if there has been a STOP on the BUS
 *          ".BusErroChk"           - Check to see if there has been an ERROR on the BUS
 *
 *          ".ClearNACK"            - Clear the NACK status bit
 *          ".ClearBusEr"           - Clear the Bus error status bis
 *          ".Clear_STOP"           - Clear the STOP status bit
 *
 *          ".TransmitEmptyITChk"   - Indicates if the interrupt for Transmit Empty has been
 *                                    enabled
 *          ".TransmitComptITChk"   - Indicates if the interrupt for Transmit Complete has been
 *                                    enabled
 *          ".ReceiveToReadITChk"   - Indicates if the interrupt for Receive buffer full has been
 *                                    enabled
 *          ".BusNACKITChk"         - Indicates if the interrupt for NACK on the I2C bus has been
 *                                    enabled
 *          ".BusSTOPITChk"         - Indicates if the interrupt for I2C Bus STOP has been enabled
 *          ".BusErrorITChk"        - Indicates if the interrupt for an I2C Bus Error has been
 *                                    enabled
 *
 *  [#] I2C Request Form System
 *      ~~~~~~~~~~~~~~~~~~~~~~~
 *      Whilst utilising interrupt based communication, the I2C Device will utilise a I2C Request
 *      Form system, so as to ensure that each of the multiple source functions wanting to
 *      communication with multiple I2C devices, is done correctly and effectively.
 *      Each of the source functions will generate a Request Form, which will then be placed into
 *      the target I2C device Form queue, such that the I2C device can go through each of these
 *      forms in sequence; each form will contain the following:
 *          Target device's I2C Address
 *          Number of packets to transmit (8bits x N)
 *          I2C Request and Mode
 *          Location for where data is to be taken/stored < This is a union, to handle
 *                                                          multiple data types
 *          Communication complete return flag      (will be updated with the amount of packets
 *                                                   transmitted successfully)
 *          I2C communication fault return flag
 *
 *      Function list (all are protected):
 *          ".GenericForm"          - Populate generic entries of the I2C Form (outputs structure)
 *          ".FormW8bitArray"       - Link form to a 8bit array location
 *          ".FormWGenBuffer"       - Link form to a GenBuffer (8bit) location
 *          ".specificRequest"      - OVERLOADED function, for putting a request onto the target
 *                                    I2C's form queue (handles both 8bit, and GenBuffer inputs)
 *
 *          ".GetFormWriteData"     - Retrieve data from I2C Form's requested location
 *          ".PutFormReadData"      - Write data to location specified by current I2C Form
 *          ".FlushFormWritedData"  - Function will wipe any remaining data from the buffer if
 *                                    I2C request was for "WRITE", and the data type is
 *                                    "GenBuffer".
 *
 *      There is no other functionality within this class.
 *************************************************************************************************/
#ifndef I2CPeriph_H_
#define I2CPeriph_H_

#include "FileIndex.h"

#include FilInd_GENBUF_HD               // Provide the template for the circular buffer class

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
// Add includes specific to the STM32Fxx devices

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================
#include "stm32l4xx_hal.h"              // Include the HAL library

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
// Defined within the class, to ensure are contained within the correct scope

class I2CPeriph {
/**************************************************************************************************
 * ==   TYPES   == >>>       TYPES GENERATED WITHIN CLASS        <<<
 *   -----------
 *  Following types are generated within this class. If needed outside of the class, need to
 *  state "I2CPeriph::" followed by the type.
 *************************************************************************************************/
public:
    enum class DevFlt : uint8_t {   // Fault Type of the class (internal enumerate)
        None            = 0x00,     // Normal Operation
        NACK            = 0x01,     // I2C No Acknowledge
        BUS_ERROR       = 0x02,     // I2C Bus error

        Initialised     = 0xFF      // Just initialised
    };

    enum class CommMode : uint8_t { // Communication mode for I2C
        AutoEnd         = 0,        // In AutoEnd mode, internal class functions will generate
                                    // STOP command, once final packet has transmitted
        Reload          = 1,        // In Reload mode, internal class will wait for more data
                                    // to be added to the buffer before continuing transmission
        SoftEnd         = 2         // In SoftEnd mode, generation of STOP/RELOAD condition
                                    // need to be done manually
    };

    enum class Request : uint8_t  { // Request mode for I2C
        Nothing         = 0,        // Do not change the state of the current communication
                                    // request
        START_WRITE     = 1,        // Start new communication, in WRITE MODE
        START_READ      = 2,        // Start new communication, in READ MODE
        STOP            = 3         // STOP current communication
    };

    enum InterState : uint8_t {ITEnable, ITDisable};// Enumerate state for enabling/disabling
                                                    // interrupts
    enum CommLock : uint8_t {Communicating, Free};  // Enumerate state for indicating if device is
                                                    // communicating

    typedef struct {            // I2C Form structure, used to manage I2C Communication interrupts
        uint16_t        devAddress;         // State the I2C Address to be used
        uint16_t        size;               // State the amount of data to be transfered

        union DataUnion  {                      // Union used to contain the data type pointer
            uint8_t               *ptr8bit;     //   8bit array
            GenBuffer<uint8_t>    *ptrGenBuf;   //   GenBuffer pointer
        } Buff;

        enum DataType : uint8_t {               // Enumerate type to support union
            Array8bit       = 0,                // Set for 8bit array (union points to 8bit array)
            GenBuffer8bit   = 1                 // Set for GenBuffer class pointer (union points
                                                // to GenBuffer)
        } BuffType;                             // Name of enumerate within structure

        Request         Reqst;                  // State the Request type
        CommMode        Mode;                   // State the Mode


        uint8_t         *Cmplt;             // Provide a pointer to a "Complete" flag (will be
                                            // incremented) - to be cleared by source function
        DevFlt          *Flt;               // Provide a pointer to a I2CPeriph::DevFlt for the
                                            // I2C fault status to be provided to source
                                            // function
    }   Form;

/**************************************************************************************************
 * == GEN PARAM == >>>       GENERIC PARAMETERS FOR CLASS        <<<
 *   -----------
 *  Parameters required for the class to function.
 *************************************************************************************************/
    protected:
        GenBuffer<Form> *FormQueue;         // Pointer to the class internal I2CForm buffer, which
                                            // is used to manage interrupt based communication.
                                            // Functions will add request forms to this buffer,
                                            // and interrupt then goes through them sequentially.
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        GenBuffer<uint8_t>  *Receive;       // Pointer to the class internal buffer for data read
                                            // back from I2C Device.
        GenBuffer<uint8_t>  *Transmit;      // Pointer to the class internal buffer for data
                                            // written to I2C Device.
        // These buffers to be used when source functionality doesn't require its own dedicated
        // buffer.
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        uint8_t         curCount;       // Current communication packet count
        Request         curReqst;       // Current request for communication (ignores "Nothing")
        Form            curForm;        // Current I2C request form

    public:
        DevFlt      Flt;                // Fault state of the I2C Device
        CommLock    CommState;          // Status of the Communication

/**************************************************************************************************
 * == SPC PARAM == >>>        SPECIFIC ENTRIES FOR CLASS         <<<
 *   -----------
 *  Following are functions and parameters which are specific for the embedded device selected.
 *  The initialisation function for the class is also within this section, which again will be
 *  different depending upon the embedded device selected.
 *************************************************************************************************/
    protected:
        void PopGenClassParameters(void);   // Populate generic parameters for the class

#if ( defined(zz__MiSTM32Fx__zz) || defined(zz__MiSTM32Lx__zz)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================
    protected:
        I2C_HandleTypeDef  *I2C_Handle;     // Store the I2C handle

public:
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#ifdef __LiteImplement__        // If "__LiteImplement__" has been defined, then need to have array
                                // fully defined, and provided to the "GenBuffer"
                                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        I2CPeriph(I2C_HandleTypeDef *I2C_Handle, GenBuffer<Form> *FormBuffer,
                GenBuffer<uint8_t> *readBuffer, GenBuffer<uint8_t> *writeBuffer);
        // Setup the UART class, for STM32 by providing the I2C type define handle, as well the
        // "GenBuffer" needing to be provided to the function, to be fully defined outside of class

#else                           // If "__LiteImplement__" has not been defined, then allow use of
                                // "new" and "delete" for defining internal arrays
                                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        UARTDevice(UART_HandleTypeDef *UART_Handle, uint32_t Buffersize);
        // Setup the UART class, for STM32Fxx by providing the UART type define handle, as well as
        // a defined value for the depth of the UART buffers

#endif                          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    public:
        I2CPeriph();

#else
//==================================================================================================
    public:
        I2CPeriph();

#endif

/**************************************************************************************************
 * == GEN FUNCT == >>>      GENERIC FUNCTIONS WITHIN CLASS       <<<
 *   -----------
 *  The following are functions scoped within the "I2CPeriph" class, which are generic; this means
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
    //void Enable(void);                      // Enable the I2C device
    //void Disable(void);                     // Disable the I2C device

    uint8_t DRRead(void);                   // Function to read direct from the hardware
    void DRWrite(uint8_t data);             // Function to write direct to the hardware

    // I2C Event status checks
    // ~~~~~~~~~~~~~~~~~~~~~~~
    uint8_t TransmitEmptyChk(void);         // Check state of transmission register (1 = empty)
    uint8_t TransmitComptChk(void);         // Check state of transmission complete (1 = cmplt)
    uint8_t ReceiveToReadChk(void);         // Check state of receive data register (1 =  read)
    uint8_t BusNACKChk(void);               // Check if No Acknowledge is received  (1 = NACK)
    void ClearNACK(void);                   // Clear the NACK bit

    uint8_t BusSTOPChk(void);               // Check if I2C bus STOP has been set   (1 = STOP)
    void Clear_STOP(void);                  // Clear the Bus STOP bit

    uint8_t BusBusyChk(void);               // Check if I2C bus is busy             (1 = BUSY)

    // I2C Error status checks
    // ~~~~~~~~~~~~~~~~~~~~~~~
    uint8_t BusErroChk(void);               // Check if START/STOP bit is set wrong (1 =  Flt)
    void ClearBusEr(void);                  // Clear the Bus error flag

    // I2C Interrupt status checks
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~
    uint8_t TransmitEmptyITChk(void);       // Check to see if interrupt is enabled (1 = enabled)
    uint8_t TransmitComptITChk(void);       // Check to see if interrupt is enabled (1 = enabled)
    uint8_t ReceiveToReadITChk(void);       // Check to see if interrupt is enabled (1 = enabled)
    uint8_t BusNACKITChk(void);             // Check to see if interrupt is enabled (1 = enabled)
    uint8_t BusSTOPITChk(void);             // Check to see if interrupt is enabled (1 = enabled)
    uint8_t BusErrorITChk(void);            // Check to see if interrupt is enabled (1 = enabled)

    // I2C Communication Request Form handling
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    void RequestTransfer(uint16_t devAddress, uint8_t size, CommMode mode, Request reqst);
        // Request a new communicate via I2C device, function handles the START/STOP, R/~W setup
        // devAddress needs to be full, i.e. 7bit address needs to be provided as 8bits. The
        // R/~W will be ignored and populated as required

    static Form GenericForm(uint16_t devAddress, uint8_t size, CommMode mode, Request reqst,
                            DevFlt *fltReturn, uint8_t *cmpFlag);

    static void FormW8bitArray(Form *RequestForm, uint8_t *pData);
    static void FormWGenBuffer(Form *RequestForm, GenBuffer<uint8_t> *genBuff);

    void FlushFormWritedData(Form *RequestForm, uint8_t count);
    // If a fault has occurred during the communication. This function is to be called, to wipe
    // any of the excess write request data - specifically if data format is "GenBuffer", as
    // other downstream functions may be utilising the same "GenBuffer" for there communication.
    // Therefore need to retain data consistency.

    void specificRequest(uint16_t devAddress, uint8_t size, uint8_t *pData,
                            CommMode mode, Request reqst, DevFlt *fltReturn, uint8_t *cmpFlag);

    void specificRequest(uint16_t devAddress, uint8_t size, GenBuffer<uint8_t> *genBuff,
                                CommMode mode, Request reqst, DevFlt *fltReturn, uint8_t *cmpFlag);

    static uint8_t GetFormWriteData(Form *RequestForm);
    // Function will retrieve the next data entry from the source data specified within the
    // I2C "RequestForm"

    static void PutFormReadData(Form *RequestForm, uint8_t readdata);
    // Function will take the data read from the I2C Hardware, and put into the requested data
    // location specified within the "RequestForm" input (will be curForm)

public:     /**************************************************************************************
             * ==  PUBLIC   == >>>    POLING FUNCTIONS FOR DATA TRANSFER     <<<
             *   -----------
             *  Visible functions used to transfer data via I2C - will wait for any registers to
             *  be in correct state before progressing.
             *************************************************************************************/
    DevFlt poleMasterTransmit(uint16_t devAddress, uint8_t *pdata, uint8_t size);
    DevFlt poleMasterReceive(uint16_t devAddress, uint8_t *pdata, uint8_t size);
    DevFlt poleDeviceRdy(uint16_t devAddress);

public:     /**************************************************************************************
             * ==  PUBLIC   == >>>   INTERRUPT FUNCTIONS FOR DATA TRANSFER   <<<
             *
             *  Visible functions used to transfer data via I2C - in Interrupt mode, so allows
             *  other functions to run, whilst hardware "does its thing!"
             *************************************************************************************/
    void configTransmtIT(InterState intr);      // Configure the Transmit Empty interrupt
    void configTransCmIT(InterState intr);      // Configure the Transmit Complete interrupt
    void configReceiveIT(InterState intr);      // Configure the Receive full interrupt
    void configBusNACKIT(InterState intr);      // Configure the NACK interrupt
    void configBusSTOPIT(InterState intr);      // Configure the BUS Stop interrupt
    void configBusErroIT(InterState intr);      // Configure the BUS Error interrupt

    void intMasterReq(uint16_t devAddress, uint8_t size, GenBuffer<uint8_t> *genBuff,
                      CommMode mode, Request reqst, DevFlt *fltReturn, uint8_t *cmpFlag);

    void I2CInterruptStart(void);           // Enable communication if bus is free, otherwise
                                            // wait (doesn't actually wait)
    void intReqFormCmplt(void);             // Closes out the input Request Form
    void IRQEventHandle(void);              // Interrupt I2C Event handler
    void IRQErrorHandle(void);              // Interrupt I2C Error handler

public:     /**************************************************************************************
             * ==  PUBLIC   == >>>    DMA FUNCTIONS FOR DATA TRANSFER    <<<
             *   -----------
             *  Visible functions used to transfer data via DMA I2C - all handling of data to the
             *  I2C hardware is managed outside of processor, and managed by dedicated hardware.
             *************************************************************************************/
    //void I2CDMAStart(void);                 // Enable communication if bus is free, via DMA
    // dmaMasterTransmit    (GenBuffer)
    // dmaMasterReceive     (GenBuffer)

    virtual ~I2CPeriph();
};

#endif /* I2CPeriph_H_ */
