/**************************************************************************************************
 * @file        SPIPeriph.h
 * @author      Thomas
 * @version     V3.3
 * @date        22 Sept 2018
 * @brief       Header file for the Generic SPIPeriph Class handle
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * Class supports multiple target devices, depending upon which one is defined will modify how the
 * SPIPeriph class can be initialised.
 * The basic used of the class is the same for all target devices
 *      Call Class SPIPeriph to initialise the class
 *          For STM32F devices, provide the address of the SPI handler - from cubeMX
 *          For STM32L devices, provide the address of the SPI handler - from cubeMX
 *          For RaspberryPi, provide the channel, speed and mode
 *
 *      Depending upon how the programmer wants to use the SPI device, will change which functions
 *      are utilised.
 *      For functions to write/read to the selected device (polling mode):
 *          ".poleMasterTransfer"   - OVERLOADED function for SPI communication
 *                                    (input varies the GPIO or DEMUX devices to select SPI, or
 *                                    hardware managed)
 *
 *      If interrupts are to be used, then will first need to be configured within the NVIC (which
 *      is not part of this class), then the following can be used:
 *          ".configTransmtIT"      - Enable/Disable Transmit Empty interrupt
 *          ".configTransCmIT"      - Enable/Disable Transmission complete interrupt
 *          ".configBusErroIT"      - Enable/Disable SPI bus errors
 *
 *          ".intMasterTransfer"    - OVERLOADED function for SPI communication in interrupt mode
 *                                    (utilises the SPI form system, see below), expects to
 *                                    receive an array data location.
 *                                    (input varies for GPIO or hardware managed Chip Select)
 *
 *          ".SPIInterruptStart"    - Check to see if the SPI bus is free, and a new request form
 *                                    is available. Then trigger a communication run (enables
 *                                    Transmit Empty/Receive interrupts)
 *          ".intReqFormCmplt"      - Function will go through tidy up procedure for the current
 *                                    Request Form
 *          ".IRQHandle"            - Functions to be placed within the relevant Interrupt Vector
 *                                    call, so as to handle the SPI interrupt
 *
 *      Following functions are protected so will only work for classes which inherit from this
 *      one, and are not visible external to this class. They contain the lower level handling
 *      of hardware, which the functions above rely upon to function (this is where a majority of
 *      the differences between the supported embedded devices will lie:
 *          ".Enable"/".Disable"    - Enabling and Disabling the SPI Peripheral
 *
 *          ".DRRead"               - Will take data straight from hardware
 *          ".DRWrite"              - Will put data straight onto the hardware
 *
 *          ".HardwareCS"           - Generate the struct as a Hardware Chip Select
 *          ".SoftwareGPIO"         - Generate the struct as a Software GPIO Chip Select
 *          ".ChipSelectHandle"     - Will handle selecting/deselecting the SPI device (using the
 *                                    above structs)
 *
 *          ".TransmitEmptyChk"     - Check to see if the Transmit Empty buffer is empty
 *          ".ReceiveToReadChk"     - Check to see if the Receive buffer is full (data to read)
 *          ".BusBusyChk"           - Check to see if the BUS is busy
 *          ".BusOverRunChk"        - Check to see if a BUS overrun has occurred
 *          ".BusModeFltChk"        - Check to see if a BUS mode fault has occurred
 *
 *          ".ClearBusOvrRun"       - Clear the BUS overrun fault (and manage the hardware)
 *          ".ClearBusModeFlt"      - Clear the BUS mode fault (and manage the hardware)
 *
 *
 *          ".TransmitEmptyITChk"   - Indicates if the interrupt for Transmit Empty has been
 *                                    enabled
 *          ".ReceiveToReadITChk"   - Indicates if the interrupt for Receive buffer full has been
 *                                    enabled
 *          ".BusErrorITChk"        - Indicates if the interrupt for Bus error has been enabled
 *
 *  [#] SPI Request Form System
 *      ~~~~~~~~~~~~~~~~~~~~~~~
 *      Whilst utilising minimised CPU loaded communication, the SPI Device will utilise a SPI
 *      Request Form system, so as to ensure that each of the multiple source functions wanting to
 *      communicate with multiple SPI devices, is done correctly and effectively.
 *      Each of the source functions will generate a Request Form, which will then be placed into
 *      the target SPI device Form queue, such that the SPI device can go through each of these
 *      forms in sequence; each form will contain the following:
 *          Target device's Chip Select management (hardware/software)
 *          Number of packets to transmit/receive (8bits x N)
 *          Location for where the data is to be taken/stored
 *          Communication complete return flag      (will be updated with the amount of packets
 *                                                   transmitted successfully)
 *          SPI communication fault return flag
 *
 *      Function list (all are protected):
 *          ".GenericForm"          - Populate generic entries of the SPI Form (outputs structure)
 *          ".FormW8bitArray"       - Link form to a 8bit array location
 *
 *          ".GetFormWriteData"     - Retrieve data from SPI Form's requested location
 *          ".PutFormReadData"      - Write data to location specified by current SPI Form
 *
 *      There is no other functionality within this class
 *************************************************************************************************/
#ifndef SPIPeriph_H_
#define SPIPeriph_H_

#include "FileIndex.h"
#include <stdint.h>

#include FilInd_GENBUF_HD               // Provide the template for the circular buffer class
#include FilInd_GPIO___HD               // Allow use of GPIO class, for Chip Select
#include FilInd_DeMux__HD

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
#include "stm32f1xx_hal.h"              // Include the HAL library

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================
#include "stm32l4xx_hal.h"              // Include the HAL library

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
#include <wiringPiSPI.h>                // Include the wiringPi SPI library

#else
//==================================================================================================
#error "Unrecognised target device"

#endif

// Defines specific within this class
// None

// Types used within this class
// Defined within the class, to ensure are contained within the correct scope

class SPIPeriph {
/**************************************************************************************************
* ==   TYPES   == >>>       TYPES GENERATED WITHIN CLASS        <<<
*   -----------
*  Following types are generated within this class. If needed outside of the class, need to
*  state "SPIDevice::" followed by the type.
*************************************************************************************************/
public:
    enum class DevFlt : uint8_t {   // Fault Type of the class (internal enumerate)
        None            = 0x00,     // Normal Operation
        ModeFault       = 0x01,     // Mode fault
        Overrun         = 0x02,     // Over run has occurred
        FrameFormat     = 0x03,     // Frame format error
        CRCError        = 0x04,     // CRC Error detected
        DataSize        = 0x05,     // Error with the size request of data

        Initialised     = 0xFF      // Just initialised
    };

    enum SPIMode : uint8_t {
        MODE0 = 0,      // Clock Idles at 0, and capture on the rising edge
        MODE1 = 1,      // Clock Idles at 0, and capture on the falling edge
        MODE2 = 2,      // Clock Idles at 1, and captures on the falling edge
        MODE3 = 3       // Clock Idles at 1, and captures on the rising edge
    };

    enum InterState : uint8_t {ITEnable, ITDisable};// Enumerate state for enabling/disabling
                                                    // interrupts
    enum CommLock : uint8_t {Communicating, Free};  // Enumerate state for indicating if device is
                                                    // communicating

    enum CSSelection : uint8_t  { Select, Deselect };
        // Enumerate type used to indicate whether device needs to be selected or deselected

    typedef struct {                // Structure to manage the multiple types of Chip Selection
                                    // options available
        GPIO    *GPIO_CS;           // Software managed GPIO CS

        enum CSType : uint8_t {     // Enumerate type to indicate how Chip Select is managed
            Hardware_Managed,       // Indicate that Hardware is manages the CS
            Software_GPIO           // Indicate that the Chip Select is software managed
        } Type;
    }   CSHandle;

    typedef struct {            // SPI Form structure, used to manage SPI Communication interrupts
        CSHandle                devLoc;     // Location to trigger the Chip Select
        uint16_t                size;       // State the amount of data to be transfered

        uint8_t                 *TxBuff;    // Pointer to data to transmit
        uint8_t                 *RxBuff;    // Pointer to data to be read back


        volatile uint16_t        *Cmplt;    // Provide a pointer to a "Complete" flag (will be
                                            // incremented) - to be cleared by source function
        volatile DevFlt          *Flt;      // Provide a pointer to a SPIPeriph::DevFlt for the
                                            // SPI fault status to be provided to source
                                            // function
    }   Form;

/**************************************************************************************************
* == GEN PARAM == >>>       GENERIC PARAMETERS FOR CLASS        <<<
*   -----------
*  Parameters required for the class to function.
*************************************************************************************************/
    protected:
    GenBuffer<Form>     FormQueue;      // Pointer to the class internal SPIForm buffer, which
                                        // is used to manage interrupt based communication.
                                        // Functions will add request forms to this buffer,
                                        // and interrupt then goes through them sequentially.
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        SPIMode     Mode;               // Selected mode of SPI Device

        uint16_t    curCount;           // Current communication packet count
        Form        curForm;            // Current SPI request form

    public:
        DevFlt      Flt;                // Fault state of the SPI Device
        CommLock    CommState;          // Status of the Communication

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
    private:
        SPI_HandleTypeDef   *SPI_Handle;    // Store the Handle for the SPI Device, from cubeMX

    public:
        SPIPeriph(SPI_HandleTypeDef *SPIHandle, Form *FormArray, uint16_t FormSize);
        // Setup the SPI class, for STM32 by providing the SPI Request Form array pointer, as well
        // as the size.
        // Class will then generate a GenBuffer item internally.

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
    private:
        int SPIChannel;                         // Store the channel used for SPI

    public:
        SPIPeriph(int channel, int speed, _SPIMode Mode);

#else
//==================================================================================================
    public
        SPIPeriph();

#endif

/**************************************************************************************************
 * == GEN FUNCT == >>>      GENERIC FUNCTIONS WITHIN CLASS       <<<
 *   -----------
 *  The following are functions scoped within the "SPIPeriph" class, which are generic; this means
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
    void Enable(void);                      // Enable the SPI device
    void Disable(void);                     // Disable the SPI device

    uint8_t DRRead(void);                   // Function to read direct from the hardware
    void DRWrite(uint8_t data);             // Function to write direct to the hardware

    // SPI Event status checks
    // ~~~~~~~~~~~~~~~~~~~~~~~
    uint8_t TransmitEmptyChk(void);         // Check state of transmission register (1 = empty)
    uint8_t ReceiveToReadChk(void);         // Check state of receive data register (1 =  read)

    uint8_t BusBusyChk(void);               // Check if SPI bus is busy             (1 = BUSY)

    // SPI Error status checks
    // ~~~~~~~~~~~~~~~~~~~~~~~
    uint8_t BusOverRunChk(void);            // Check state of data over run         (1 = ovrun)
    void ClearBusOvrRun(void);              // Clear the Bus Over run fault

    uint8_t BusModeFltChk(void);            // Check if there has been a mode fault (1 = fault)
    void ClearBusModeFlt(void);             // Clear the Bus Mode fault

    // SPI Interrupt status checks
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~
    uint8_t TransmitEmptyITChk(void);       // Check to see if interrupt is enabled (1 = enabled)
    uint8_t ReceiveToReadITChk(void);       // Check to see if interrupt is enabled (1 = enabled)
    uint8_t BusErrorITChk(void);            // Check to see if interrupt is enabled (1 = enabled)

    // SPI Chip Selection management
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    CSHandle HardwareCS(void);
    CSHandle SoftwareGPIO(GPIO *CS);

    void ChipSelectHandle(CSHandle selection, CSSelection Mode);

    // SPI Communication Request Form handling
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    static Form GenericForm(CSHandle devLoc, uint16_t size,
                            volatile DevFlt *fltReturn, volatile uint16_t *cmpFlag);

    static void FormW8bitArray(Form *RequestForm, uint8_t *TxData, uint8_t *RxData);

    static uint8_t GetFormWriteData(Form *RequestForm);
    // Function will retrieve the next data entry from the source data specified within the
    // SPI "RequestForm"

    static void PutFormReadData(Form *RequestForm, uint8_t readdata);
    // Function will take the data read from the SPI Hardware, and put into the requested data
    // location specified within the "RequestForm" input

public:     /**************************************************************************************
             * ==  PUBLIC   == >>>    POLING FUNCTIONS FOR DATA TRANSFER     <<<
             *   -----------
             *  Visible functions used to transfer data via SPI - will wait for any registers to
             *  be in correct state before progressing.
             *************************************************************************************/
    DevFlt poleMasterTransfer(uint8_t *wData, uint8_t *rData, uint16_t size);
    DevFlt poleMasterTransfer(GPIO *ChipSelect, uint8_t *wData, uint8_t *rData, uint16_t size);
    DevFlt poleMasterTransfer(DeMux *DeMuxCS, uint8_t CSNum,
                              uint8_t *wData, uint8_t *rData, uint16_t size);

public:     /**************************************************************************************
             * ==  PUBLIC   == >>>   INTERRUPT FUNCTIONS FOR DATA TRANSFER   <<<
             *
             *  Visible functions used to transfer data via SPI - in Interrupt mode, so allows
             *  other functions to run, whilst hardware "does its thing!"
             *************************************************************************************/
    void configTransmtIT(InterState intr);      // Configure the Transmit Empty interrupt
    void configReceiveIT(InterState intr);      // Configure the Receive full interrupt
    void configBusErroIT(InterState intr);      // Configure the BUS Error interrupt

    void intMasterTransfer(uint16_t size, uint8_t *TxBuff, uint8_t *RxBuff,
                           volatile DevFlt *fltReturn, volatile uint16_t *cmpFlag);

    void intMasterTransfer(GPIO *CS, uint16_t size, uint8_t *TxBuff, uint8_t *RxBuff,
                           volatile DevFlt *fltReturn, volatile uint16_t *cmpFlag);
    // Above OVERLOADED function "intMasterTransfer" takes the input parameters and uses this to
    // populate a SPI Request Form, and then add this to the Device Queue.

    void SPIInterruptStart(void);               // Enable communication if bus is free, otherwise
                                                // wait (doesn't actually wait)
    void intReqFormCmplt(void);                 // Closes out the input Request Form
    void IRQHandle(void);                       // Interrupt Handle for SPI Device

        virtual ~SPIPeriph();
};

#endif /* SPIPeriph_H_ */
