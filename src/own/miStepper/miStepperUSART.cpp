/**************************************************************************************************
 * @file        miStepperUSART.cpp
 * @author      Thomas
 * @brief       << Manually Entered >>
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include <FileIndex.h>                  // Header for miLibrary index
#include FilIndMStpUARTHD               // Header for SPI

// C System Header(s)
// ------------------
#include <stdint.h>

// C++ System Header(s)
// --------------------

// Other Libraries
// --------------
#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
// None

#elif (zz__MiEmbedType__zz == 51)       // If the target device is an STM32Lxx from cubeMX then
//=================================================================================================
// None

#elif (zz__MiEmbedType__zz == 10)       // If the target device is an Raspberry Pi then
//=================================================================================================
// None

#else
//=================================================================================================
#error "Unrecognised target device"

#endif

// Project Libraries
// -----------------
#include FilInd_GENBUF_TP               // Provide the template for the circular buffer class
#include FilInd_USART__HD               // Include the USART Class handler
#include FilIndUSARTDMAHD               // Include the USART DMA specific class
#include FilInd_DATMngrHD               // Provide the function set for Data Manipulation

//=================================================================================================
static uint16_t mistepper_crc_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
};

miStepperUSART::miStepperUSART(uint8_t *out_array, uint16_t out_array_size,
                               uint8_t *in_array,  uint16_t in_array_size) {
/**************************************************************************************************
 * Construct the miStepperUSART, and populate with the externally defined arrays for message out,
 * and in.
 * Initialise the state machine(s) to "IDLE"/"FREE"/, and clear the message pointer
 *************************************************************************************************/
    _message_out_.create(out_array, out_array_size);
    _message_in_.create(in_array,   in_array_size);

    _state_             =   RdState::kIdle;
    _message_in_start_  = 0;

    // PC source signals
    // ==================
    mode        = 0x00;

    // Device source signals
    // ======================
    packet_count                    = 0;

    // SPI parameters
    angular_position                = 0;
    spi1_fault                      = 0;
    angle_sensor_spi_fault          = 0;
    angle_sensor_fault              = 0;
    angle_sensor_idle_count         = 0;
    spi1_task_time                  = 0;

    // I2C parameters (top only)
    internal_temperature_top        = 0;
    i2c1_fault                      = 0;
    top_temp_sensor_i2c_fault       = 0;
    top_temp_sensor_fault           = 0;
    top_temp_sensor_idle_count      = 0;
    i2c1_task_time                  = 0;

    // ADC parameters
    internal_voltage_reference      = 0;
    cpu_temperature                 = 0;
    fan_voltage                     = 0;
    fan_current                     = 0;
    stepper_voltage                 = 0;
    stepper_current                 = 0;
    conversion_fault                = 0;
    adc1_task_time                  = 0;

    // Fan Parameters
    fan_demand                      = 0;
    fan_task_time                   = 0;

    // Stepper Parameters
    stepper_frequency               = 0;
    stepper_gear                    = 0;
    stepper_calc_position           = 0;
    stepper_task_time               = 0;

    // USART Parameters
    usart1_task_time                = 0;


}

void miStepperUSART::updateReadStateMachine(uint8_t data_value, uint16_t position) {
/**************************************************************************************************
 * Manage the state machine:
 *      1-> IDLE        : Will listen on the USART till the initial parameter is captured
 *                        ('kdial_tone'). Once this is confirmed, move to CONFIRMING
 *      2-> CONFIRMING  : Will now check for the 'ksource_key', then transition to LISTENING.
 *                        If not 'ksource_key' then return to IDLE
 *      3-> LISTENING   : Valid data to be read to be downloaded. Once expected size of data
 *                        has been read - then check CRC
 *************************************************************************************************/
    if      ( (_state_ == RdState::kIdle)       && (data_value == kdial_tone) ) {
        _message_in_start_ = position;
        _state_ = RdState::kConfirming;
    }
    else if ( (_state_ == RdState::kConfirming) && (data_value == kread_key) ) {

        _state_ = RdState::kListening;
    }
    else {
        _state_ = RdState::kIdle;
    }
}

uint16_t miStepperUSART::update_crc(uint16_t crc_accum,
                                    uint8_t *data_blk_ptr, uint16_t data_blk_size) {
/**************************************************************************************************
 * Calculate the crc of the miStepperUSART
 *************************************************************************************************/
    uint16_t i, j;

    for (j = 0; j < data_blk_size; j++)
    {
        i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ mistepper_crc_table[i];
    }

    return crc_accum;
}

uint16_t miStepperUSART::update_crc(uint16_t crc_accum, GenBuffer<uint8_t> *data_blk_ptr,
                                    uint16_t message_start, uint16_t data_blk_size) {
/**************************************************************************************************
 * Calculate the crc of the miStepperUSART (overloaded function)
 * This version makes use of the GenBuffer, starting the crc calculation from position
 * '_message_in_start_', upto 'data_blk_size'.
 *************************************************************************************************/
    uint16_t i, j;

    for (j = 0; j < data_blk_size; j++)
    {
        i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr->pa[( (_message_in_start_ + j) %
                                                              _message_in_.length )])
            & 0xFF;

        crc_accum = (crc_accum << 8) ^ mistepper_crc_table[i];
    }

    return crc_accum;
}

void miStepperUSART::decodeMessage(void) {
/**************************************************************************************************
 * Will check for any new input messages, then feed it through the state machine till the correct
 * message configuration is provided.
 *************************************************************************************************/
    uint8_t     read_back = 0x00;   // Variable for retrieving the data from array
    uint16_t    temp_value = _message_in_.output_pointer;
        // Store the current position of GenBuffer (so as to save on calculating the back step if
        // the '_message_in_' is part of a valid message

    while(_message_in_.outputRead(&read_back) != kGenBuffer_Empty) {
        if (_state_ != RdState::kListening) {
            updateReadStateMachine(read_back, temp_value);
        }
        else {
            temp_value = (uint16_t) ( ( _message_in_.length + _message_in_.output_pointer
                                          - _message_in_start_ )
                                      % _message_in_.length);
            /* Calculate the size of data read in; by taking the delta of the message start -
             * '_message_in_start_', and the current read data from buffer.
             * Rest of calculation takes into account of buffer size
             */

            if (temp_value >= 18) {
                _state_ = RdState::kIdle;

                if (update_crc(0, &_message_in_, _message_in_start_, 18) == 0) {
                    temp_value = (uint16_t) ( (_message_in_start_ + 3) % _message_in_.length );

                    mode = _message_in_.pa[ ( (_message_in_start_ + 3) % _message_in_.length ) ];

            }}
        }

        temp_value = _message_in_.output_pointer;
    }
}

void miStepperUSART::decodeMessageIN(UARTPeriph    *usart_handle,
                                     volatile UARTPeriph::DevFlt *fltReturn,
                                     volatile uint16_t *cmpFlag) {
/**************************************************************************************************
 * OVERLOADED function setting the linked 'usart_handle' to "readGenBufferLock" (normal UART
 * peripheral)
 *************************************************************************************************/
    usart_handle->readGenBufferLock(&_message_in_,  fltReturn,  cmpFlag);
    decodeMessage();
}

void miStepperUSART::decodeMessageIN(UARTDMAPeriph *usart_handle,
                                     volatile UARTPeriph::DevFlt *fltReturn,
                                     volatile uint16_t *cmpFlag) {
/**************************************************************************************************
 * OVERLOADED function setting the linked 'usart_handle' to "readGenBufferLock" (DMA UART
 * peripheral)
 *************************************************************************************************/
    usart_handle->readGenBufferLock(&_message_in_,  fltReturn,  cmpFlag);
    decodeMessage();
}

void miStepperUSART::encodeMessage(uint8_t position, uint8_t  data) {
/**************************************************************************************************
 * Will add the contents of 'data' to buffer.
 *************************************************************************************************/
    _message_out_.pa[position]  =  data;
}

void miStepperUSART::encodeMessage(uint8_t position, uint16_t  data) {
/**************************************************************************************************
 * Will add the contents of 'data' to buffer.
 *************************************************************************************************/
    DataManip::_16bit_2_2x8bit(data,  &_message_out_.pa[position]);
}

void miStepperUSART::encodeMessage(uint8_t position, uint32_t  data) {
/**************************************************************************************************
 * Will add the contents of 'data' to buffer.
 *************************************************************************************************/
    DataManip::_32bit_2_4x8bit(data,  &_message_out_.pa[position]);
}

void miStepperUSART::sendEncodeMessageOUT(void) {
/**************************************************************************************************
 * Function will generate the required packet and structure for transmission out of device
 *************************************************************************************************/
    _message_out_.qFlush(); // Flush contents of buffer, to ensure that entry [0] is starting
                            // point.
    encodeMessage(0x00,  kdial_tone                     );
    encodeMessage(0x01,  ktransmit_key                  );

    encodeMessage(0x02,  packet_count                   );

    // SPI parameters
    encodeMessage(0x04,  angular_position               );
    encodeMessage(0x08,  spi1_fault                     );
    encodeMessage(0x09,  angle_sensor_spi_fault         );
    encodeMessage(0x0A,  angle_sensor_fault             );
    encodeMessage(0x0B,  angle_sensor_idle_count        );
    encodeMessage(0x0C,  spi1_task_time                 );

    // I2C parameters (top only)
    encodeMessage(0x10,  internal_temperature_top       );
    encodeMessage(0x14,  i2c1_fault                     );
    encodeMessage(0x15,  top_temp_sensor_i2c_fault      );
    encodeMessage(0x16,  top_temp_sensor_fault          );
    encodeMessage(0x17,  top_temp_sensor_idle_count     );
    encodeMessage(0x18,  i2c1_task_time                 );

    // ADC parameters
    encodeMessage(0x1C,  internal_voltage_reference     );
    encodeMessage(0x20,  cpu_temperature                );
    encodeMessage(0x24,  fan_voltage                    );
    encodeMessage(0x28,  fan_current                    );
    encodeMessage(0x2C,  stepper_voltage                );
    encodeMessage(0x30,  stepper_current                );

    encodeMessage(0x37,  conversion_fault               );
    encodeMessage(0x38,  adc1_task_time                 );

    // Fan Parameters
    encodeMessage(0x3C,  fan_demand                     );
    encodeMessage(0x40,  fan_task_time                  );

    // Stepper Parameters
    encodeMessage(0x44,  stepper_frequency              );
    encodeMessage(0x46,  stepper_gear                   );
    encodeMessage(0x48,  stepper_calc_position          );
    encodeMessage(0x4C,  stepper_task_time              );

    // USART Parameters
    encodeMessage(0x50,  usart1_task_time               );

    uint16_t crc_value = update_crc(0, &_message_out_.pa[0],
                                    84);

    encodeMessage(0x54,  crc_value);

    _message_out_.input_pointer = 0x56;
}

void miStepperUSART::sendEncodeMessageOUT(UARTPeriph    *usart_handle,
                                          volatile UARTPeriph::DevFlt *fltReturn,
                                          volatile uint16_t *cmpFlag) {
/**************************************************************************************************
 * OVERLOADED function setting the linked 'usart_handle' to transmit the contents of 'message_out'
 *************************************************************************************************/
    sendEncodeMessageOUT();     // Calculate the CRC

    usart_handle->intWrtePacket(&_message_out_.pa[0], _message_out_.input_pointer,
                                fltReturn, cmpFlag);
}

void miStepperUSART::sendEncodeMessageOUT(UARTDMAPeriph *usart_handle,
                                          volatile UARTPeriph::DevFlt *fltReturn,
                                          volatile uint16_t *cmpFlag) {
/**************************************************************************************************
 * OVERLOADED function setting the linked 'usart_handle' to transmit the contents of 'message_out'
 *************************************************************************************************/
    sendEncodeMessageOUT();     // Calculate the CRC

    usart_handle->intWrtePacket(&_message_out_.pa[0], _message_out_.input_pointer,
                                fltReturn, cmpFlag);
}

miStepperUSART::~miStepperUSART()
{
    // TODO Auto-generated destructor stub
}
