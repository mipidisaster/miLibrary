/**************************************************************************************************
 * @file        UARTDMAPeriph.h
 * @author      Thomas
 * @version     V0.1
 * @date        28 Sep 2019
 * @brief       Header file for the Generic UART with DMA Class handle
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * This class combines together 'UARTPeriph' and the 'DMAPeriph' classes, granting the capability
 * to utilise DMAs for UART communication within the embedded device - reducing the CPU load.
 * This class only supports embedded devices which have a DMA connected to the UART, supported
 * devices:
 *  STM32
 *
 * The class will allow capability to use either DMA control of Receive/Transmit/Both; assumption
 * is that if a single DMA is used, the top level functionality will not attempt to manage that
 * communication, e.g. DMA is configured for Receive, the top level functions will therefore NOT
 * attempt to response to any 'Receive Empty' interrupt flags
 *
 *      The following 'UARTPeriph' functions will be redefined for use with DMA:
 *          ".UARTInterruptStart"   - This will update the Interrupt setup function with switches
 *                                    for DMA enabling (either Transmit/Receive). If DMA is
 *                                    disabled, logic is the same as 'UARTPeriph', if DMA is
 *                                    enabled, then the relevant DMA is configured to manage
 *                                    communication.
 *            #####################################################################################
 *            ## NOTE-> If DMA is used for Receive, class assumes that the DMA has been configured
 *            ##        for circular mode, and will therefore NOT enable any transfer complete
 *            ##        interrupts; as data will ALWAYS be pushed to same array
 *            #####################################################################################
 *          ".intWrteFormCmplt"     - Will tidy up the current write request, with DMA enabled
 *                                    it will retrieve number of data points transmitted from DMA
 *                                    registers
 *          ".intReadFormCmplt"     - Will tidy up the current read request, with DMA enabled
 *                                    it will retrieve number of data points transmitted from DMA
 *                                    registers
 *          ".Read_GenBufferLock"   - Function assumes that the DMA linked to Receive will be in
 *                                    circular mode, will then determine how many data points
 *                                    received and update linked 'GenBuffer'
 * 
 *      The following functions will be added, so as to manage the DMA interrupts
 *          ".IRQDMATxHandle"       - Functions to be placed within the relevant Interrupt Vector
 *                                    call, for DMA linked to Transmit, so as to handle interrupts
 *          ".IRQDMARxHandle"       - Functions to be placed within the relevant Interrupt Vector
 *                                    call, for DMA linked to Receive, so as to handle interrupts
 *
 *      There is no other functionality within this class.
 *************************************************************************************************/
#ifndef UARTDMAPeriph_H_
#define UARTDMAPeriph_H_

#include "FileIndex.h"
#include <stdint.h>

#include FilInd_USART__HD
#include FilInd_DMAPe__HD

#if   defined(zz__MiSTM32Fx__zz)        // If the target device is an STM32Fxx from cubeMX then
//==================================================================================================
#include "stm32f1xx_hal.h"              // Include the HAL library

#elif defined(zz__MiSTM32Lx__zz)        // If the target device is an STM32Lxx from cubeMX then
//==================================================================================================
#include "stm32l4xx_hal.h"              // Include the HAL library

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//==================================================================================================
#error "Unsupported target device"

#else
//==================================================================================================
#error "Unrecognised target device"

#endif

// Defines specific within this class
// None

// Types used within this class
// Defined within the class, to ensure are contained within the correct scope

class UARTDMAPeriph : public UARTPeriph, public DMAPeriph {
/**************************************************************************************************
 * ==   TYPES   == >>>       TYPES GENERATED WITHIN CLASS        <<<
 *   -----------
 *  Following types are generated within this class. If needed outside of the class, need to
 *  state "UARTDMAPeriph::" followed by the type.
 *************************************************************************************************/
// None (inherited from both UARTPeriph and DMAPeriph)

/**************************************************************************************************
* == GEN PARAM == >>>       GENERIC PARAMETERS FOR CLASS        <<<
*   -----------
*  Parameters required for the class to function.
*************************************************************************************************/
    protected:
        DMA_HandleTypeDef   *DMA_Rx;            // Store the UART DMA handle (Receive)
        DMA_HandleTypeDef   *DMA_Tx;            // Store the UART DMA handle (Transmit)

        DMAMode             ModeTx;             // DMA Transmit Communication Mode
        DMAMode             ModeRx;             // DMA Receive  Communication Mode

/**************************************************************************************************
 * == SPC PARAM == >>>        SPECIFIC ENTRIES FOR CLASS         <<<
 *   -----------
 *  Following are functions and parameters which are specific for the embedded device selected.
 *  The initialisation function for the class is also within this section, which again will be
 *  different depending upon the embedded device selected.
 *************************************************************************************************/

public:
    UARTDMAPeriph(UART_HandleTypeDef *UART_Handle, Form *WrteForm, uint16_t WrteFormSize,
                  Form *ReadForm, uint16_t ReadFormSize,
                  DMA_HandleTypeDef *DMA_Rx_Handle, DMA_HandleTypeDef *DMA_Tx_Handle);

/**************************************************************************************************
 * == GEN FUNCT == >>>      GENERIC FUNCTIONS WITHIN CLASS       <<<
 *   -----------
 *  The following are functions scoped within the "UARTDMAPeriph" class, which are generic; this
 *  means are used by ANY of the embedded devices supported by this class.
 *  The internals of the class, will then determine how it will be managed between the multiple
 *  embedded devices.
 *************************************************************************************************/
public:     /**************************************************************************************
             * ==  PUBLIC   == >>>      DMA FUNCTIONS FOR DATA TRANSFER      <<<
             *
             *  Following functions are redefinitions of 'UARTPeriph', functions such as to allow
             *  interfacing with DMAs
             *************************************************************************************/
    void UARTInterruptStart(void);      // Enable communication is bus is free, otherwise
                                        // wait (doesn't actually pause at this point)

    void intWrteFormCmplt(void);        // Closes out the input Request Form (Write)
    void intReadFormCmplt(void);        // Closes out the input Request Form (Read)

    void Read_GenBufferLock(GenBuffer<uint8_t> *ReadArray,
                            volatile DevFlt *fltReturn, volatile uint16_t *cmpFlag);

// USART/DMA specific functions
    void IRQDMATxHandle(void);            // Interrupt handler for DMA Transmit
    void IRQDMARxHandle(void);            // Interrupt handler for DMA Receive

    virtual ~UARTDMAPeriph();
};

#endif /* UARTDMAPERIPH_H_ */
