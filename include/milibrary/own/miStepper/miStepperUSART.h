/**************************************************************************************************
 * @file        miStepperUSART.h
 * @author      Thomas
 * @brief       << Manually Entered >>
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * Specific class for handling the protocol used for USART communication between the miStepper
 * device, and the user PC.
 * The class, will:
 *      Read all the input data via USART, and determine if there is a valid packet of data that
 *      has been read.
 *
 *      Will populate the required data to be transmitted to the user PC.
 *
 * Data packet size, and location is as per "USARTTransmission" (git version 'v-.-.-')
 *************************************************************************************************/
#ifndef MISTEPPERUSART_H_
#define MISTEPPERUSART_H_

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
// Add includes specific to the STM32Fxx devices

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//================================================================================================
// Add includes specific to the STM32Lxx devices

#elif (zz__MiEmbedType__zz == 10)       // If the target device is an Raspberry Pi then
//=================================================================================================
// Add includes specific to the Raspberry Pi

#elif (defined(zz__MiEmbedType__zz)) && (zz__MiEmbedType__zz ==  0)
//     If using the Linux (No Hardware) version then
//=================================================================================================
// None

#else
//=================================================================================================
#error "Unrecognised target device"

#endif

// Project Libraries
// -----------------
#include FilInd_GENBUF_TP               // Allow use of GenBuffer template class
#include FilInd_USART__HD               // Include the USART Class handler
#include FilIndUSARTDMAHD               // Include the USART DMA specific class

//=================================================================================================

// Defines specific within this class
// None
// \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/

class miStepperUSART {
public:
    static const uint8_t    kdial_tone          = 0xFF;
#ifdef zz__miStepper__zz
    static const uint8_t    kread_key           = 0xA5;
    static const uint8_t    ktransmit_key       = 0x5A;
#else
    static const uint8_t    kread_key           = 0x5A;
    static const uint8_t    ktransmit_key       = 0xA5;
#endif
    /* The transmission from the device (be it the miStepper or PC), has a specific code at the
     * start. The above captures this as a constant value
     */

    // Enabling bits for configuration within miStepper embedded device
    static const uint8_t    kenable_transmit      = 0x01;
    static const uint8_t    kreset_packetcount    = 0x02;
    static const uint8_t    kenable_interface     = 0x08;

public:
    // PC Source signals
    // =================
        uint8_t         mode;               // Internal mode register.

    // Device Source sig
    // =================
        uint16_t    packet_count;


        // SPI parameters
        uint32_t    angular_position;
        uint8_t     spi1_fault;
        uint8_t     angle_sensor_spi_fault;
        uint8_t     angle_sensor_fault;
        uint8_t     angle_sensor_idle_count;
        uint32_t    spi1_task_time;

        // I2C parameters (top only)
        uint32_t    internal_temperature_top;
        uint8_t     i2c1_fault;
        uint8_t     top_temp_sensor_i2c_fault;
        uint8_t     top_temp_sensor_fault;
        uint8_t     top_temp_sensor_idle_count;
        uint32_t    i2c1_task_time;

        // ADC parameters
        uint32_t    internal_voltage_reference;
        uint32_t    cpu_temperature;
        uint32_t    fan_voltage;
        uint32_t    fan_current;
        uint32_t    stepper_voltage;
        uint32_t    stepper_current;
        uint32_t    conversion_fault;
        uint32_t    adc1_task_time;

        // Fan Parameters
        uint32_t    fan_demand;
        uint32_t    fan_task_time;

        // Stepper Parameters
        uint16_t    stepper_frequency;
        uint16_t    stepper_gear;
        uint32_t    stepper_calc_position;
        uint32_t    stepper_task_time;

        // USART Parameters
        uint32_t    usart1_task_time;

/**************************************************************************************************
 * ==   TYPES   == >>>       TYPES GENERATED WITHIN CLASS        <<<
 *   -----------
 *  Following types are generated within this class. If needed outside of the class, need to
 *  state "miStepperUSART::" followed by the type.
 *************************************************************************************************/
protected:
    enum RdState : uint8_t {kIdle, kConfirming, kListening};
    /* State machine to manage communication:
     *      1-> IDLE        : Will listen on the USART till the initial parameter is captured
     *                        ('kdial_tone'). Once this is confirmed, move to CONFIRMING
     *      2-> CONFIRMING  : Will now check for the 'ksource_key', then transition to LISTENING.
     *                        If not 'ksource_key' then return to IDLE
     *      3-> LISTENING   : Valid data to be read to be downloaded. Once expected size of data
     *                        has been read - then check CRC
     */

/**************************************************************************************************
 * == GEN PARAM == >>>       GENERIC PARAMETERS FOR CLASS        <<<
 *   -----------
 *  Parameters required for the class to function.
 *************************************************************************************************/
    RdState         _state_;            // State of the reading of data from external devices

    uint16_t        _message_in_start_; // Pointer to where valid data is within the 'message_in'
                                        // array

    GenBuffer<uint8_t> _message_out_;   // GenBuffer for data to be sent OUT of target device
    GenBuffer<uint8_t> _message_in_;    // GenBuffer for data to be read into TARGET device

/**************************************************************************************************
 * == SPC PARAM == >>>        SPECIFIC ENTRIES FOR CLASS         <<<
 *   -----------
 *  Following are functions and parameters which are specific for the embedded device selected.
 *  The initialisation function for the class is also within this section, which again will be
 *  different depending upon the embedded device selected.
 *  >> NOTE <<
 *      As this is a class which utilises the lower level classes, the selection of the embedded
 *      device at this level doesn't change how the class works, therefore there is no selection
 *      of different devices.
 *************************************************************************************************/
#ifdef zz__miStepper__zz
public:
    miStepperUSART(uint8_t *out_array, uint16_t out_array_size,
                   uint8_t *in_array,  uint16_t in_array_size);


#else

#endif

    virtual ~miStepperUSART();

/**************************************************************************************************
 * == GEN FUNCT == >>>      GENERIC FUNCTIONS WITHIN CLASS       <<<
 *   -----------
 *  The following are functions scoped within the "miStepperUSART" class, which are generic; this
 *  means are used by ANY of the embedded devices supported by this class.
 *  >> NOTE <<
 *      As this is a class which utilises the lower level classes, the selection of the embedded
 *      device at this level doesn't change how the class works, therefore there is no selection
 *      of different devices.
 *************************************************************************************************/
protected:  /**************************************************************************************
             * == PROTECTED == >>>  FUNDAMENTAL FUNCTION FOR CLASS TO WORK   <<<
             *   -----------
             *  These functions are the bases for use of this class. The "upper level" function
             *  (which are public), rely upon these functions to operate.
             *  Are protected, as the upper level functions will not need to use these.
             *************************************************************************************/
    void updateReadStateMachine(uint8_t data_value, uint16_t position);
    uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);
    uint16_t update_crc(uint16_t crc_accum, GenBuffer<uint8_t> *data_blk_ptr,
                        uint16_t message_start, uint16_t data_blk_size);

    void decodeMessage(void);       // Function to read decode the '_message_in_' array for any
                                    // messages

    void sendEncodeMessageOUT(void);    // Function will then transmits the stored content of
                                        // '_message_out_', covering the crc as well.

    void encodeMessage(uint8_t position, uint8_t   data);
    void encodeMessage(uint8_t position, uint16_t  data);
    void encodeMessage(uint8_t position, uint32_t  data);
    // OVERLOADED will take the input 'data' and add this onto the '_message_out_' array.

public:     /**************************************************************************************
             * ==  PUBLIC   == >>>    FUNCTIONS FOR CODING/DECODING DATA     <<<
             *   -----------
             *  Visible functions used to read in the read data, and decode what has been
             *  transmitted to targetted device.
             *  AS well as functions used to populate the array to be transmitted from device
             *************************************************************************************/
    void decodeMessageIN(UARTPeriph    *usart_handle,
                         volatile UARTPeriph::DevFlt *fltReturn, volatile uint16_t *cmpFlag);
    void decodeMessageIN(UARTDMAPeriph *usart_handle,
                         volatile UARTPeriph::DevFlt *fltReturn, volatile uint16_t *cmpFlag);
    // OVERLOADED configure the linked 'usart_handle' to "readGenBufferLock", with the
    // '_message_in_' buffer
    // Note - calls then calls the 'decodeMessage' function


    void sendEncodeMessageOUT(UARTPeriph    *usart_handle,
                              volatile UARTPeriph::DevFlt *fltReturn, volatile uint16_t *cmpFlag);
    void sendEncodeMessageOUT(UARTDMAPeriph *usart_handle,
                              volatile UARTPeriph::DevFlt *fltReturn, volatile uint16_t *cmpFlag);
    // OVERLOADED will take the 'usart_handle' and link the contents of '_message_out_' for
    // transmission
};

#endif /* MISTEPPERUSART_H_ */
