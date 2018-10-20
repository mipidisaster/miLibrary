/**************************************************************************************************
 * @file        AS5x4x.cpp
 * @author      Thomas
 * @version     V0.1
 * @date        20 Oct 2018
 * @brief       Source file for the AMS Angular Position device (AS5x4x)
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include "FileIndex.h"
#include FilInd_AS5x4__HD

uint8_t AS5x4x::EvenParityCheck(uint16_t packet) {
/**************************************************************************************************
 * Function will determine if the input data is an even parity
 * This is done by going through the bits set from the input data, then take this number and put
 * through the "%" (modulus) function, which if the value is divisible by 2 will return 0 = EVEN
 * parity
 * Otherwise will not return 0 = ODD parity
 *************************************************************************************************/
    uint8_t count = 0, i = 0;

    for (i = 0; i != 16; i++) {         // Cycle through each bit in the data packet
        if (packet & 0x01) count++;     // If the lower bit is "1", increment count
        packet >>= 1;                   // Shift data down by 1 (/2)
    }

    if (count % 2 == 0)         // If fully divisible by 2, then value is even parity
        return(1);              // return 1 - value is EVEN
    else                        // Otherwise
        return(0);              // return 0 - value is ODD
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#ifdef __LiteImplement__        // If "__LiteImplement__" has been defined, then need to have array
                                // fully defined, and provided to the "GenBuffer"
                                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
AS5x4x::AS5x4x(_AS5x4xDev Device, GenBuffer<uint16_t> *wtBuff, GenBuffer<uint16_t> *rdBuff) {
/**************************************************************************************************
 * Create a AS5x4x class. For construction the of the class, the Device type needs to be provided,
 * along with a pointer to the Write and Read buffers, which will be used internally to the class
 * to manage the data that is transfered to the attached device.
 * The class construction will initialise all the parameters, and set the fault state to
 * initialised.
 *************************************************************************************************/

    this->Angle         = -999;                 // All parameters set to zero
    this->AngularSteps  = 0;                    //
    this->Mag           = 0;                    //
    this->AGC           = 0;                    //

    this->Flt           = AS5x4x_Initialised;   // Set fault state to initialised
    this->Diagnostic    = 0;                    // Initialise Diagnostic flags to zero
    this->Device        = Device;               // Store the device

    // Setup buffer pointers:
    this->wtBuff        = wtBuff;               // Link GenBuffer to write buffer
    this->rdBuff        = rdBuff;               // Link GenBuffer to read buffer

    this->wtBuff->Flush();                      // Flush data
    this->rdBuff->Flush();                      // Flush data
}

#else                           // If "__LiteImplement__" has not been defined, then allow use of
                                // "new" and "delete" for defining internal arrays
                                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
AS5x4x::AS5x4x(_AS5x4xDev Device, uint32_t Buffersize) {
/**************************************************************************************************
 * Create a AS5x4x class. For construction the of the class, the Device type needs to be provided.
 * The class construction will initialise all the parameters, and set the fault state to
 * initialised.
 *************************************************************************************************/

    this->Angle         = -999;                 // All parameters set to zero
    this->AngularSteps  = 0;                    //
    this->Mag           = 0;                    //
    this->AGC           = 0;                    //

    this->Flt           = AS5x4x_Initialised;   // Set fault state to initialised
    this->Diagnostic    = 0;                    // Initialise Diagnostic flags to zero
    this->Device        = Device;               // Store the device

    // Setup buffer pointers:
    this->wtBuff        = new GenBuffer<uint16_t>(Buffersize);
    this->rdBuff        = new GenBuffer<uint16_t>(Buffersize);
}
#endif                          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void AS5x4x::WriteDataPacket(uint16_t PacketData) {
/**************************************************************************************************
 * Function will take the input packet to be transmitted to the AS5x4x device.
 * Prior to putting into the internal write buffer, it will be checked to ensure the data is of
 * EVEN parity.
 *************************************************************************************************/
    if (this->EvenParityCheck(PacketData) != 1)         // If not EVEN parity then
        PacketData |= AS5x4x_PARITY;                    // make data EVEN parity

    this->wtBuff->InputWrite(PacketData);               // Put data into the buffer
}

void AS5x4x::ReadDataPacket(void) {
/**************************************************************************************************
 * Function will cycle through the internal read buffer for the AS5x4x device.
 * As the way to understand the read data is based upon what was written to the device (offset by
 * 1). The function will look at the previous transmission to the device to determine how to
 * deconstruct the read data.
 * Depending upon which device the AS5x4x class has been configured for, this function will call
 * another sub-function to complete the deconstruction of the data.
 *************************************************************************************************/
    uint32_t sentpacket = 0;            // Variable to store the data/command sent to the AS5x4x
                                        // for which the read data is linked (off by 1)
    uint16_t tempreq = 0;               // Variable to store the 16bit temporarily (Request)
    uint16_t tempread = 0;              // Variable to store the 16bit temporarily (Read back)

    _GenBufState ExitLoop = this->rdBuff->State();      // Get current state of buffer

    while(ExitLoop != GenBuffer_Empty) {
        // Retrieve the position of the current readback, and then loop back a previous entry
        if (this->rdBuff->output_pointer == 0)          // If start of buffer
            sentpacket = this->rdBuff->length - 1;      // Look at end of buffer
        else                                            // Otherwise loop back
            sentpacket = (this->rdBuff->output_pointer - 1) % this->rdBuff->length;

        tempreq = this->wtBuff->pa[sentpacket];         // Retrieve the requested data
        ExitLoop = this->rdBuff->OutputRead(&tempread); // Read back the Read data (and determine
                                                        // state of buffer)

        if (this->EvenParityCheck(tempread) != 1)       // If the read data is not EVEN parity
            this->Flt = AS5x4x_Parity;                  // Set the fault to "Parity" fault

        else {                                          // If the read data is EVENT then
            tempreq &= AS5x4x_DataMask;                 // Only get the address data
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            if (this->Device == AS5048A) {                      // If device is AS5048A
                this->deconstructAS5048A(tempreq, tempread);    // Call AS5048A deconstruction
            }
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            else if (this->Device == AS5047D) {                 // If device is AS5047D
                this->deconstructAS5047D(tempreq, tempread);    // Call AS5047D deconstruction
            }
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        }
    }
}

uint16_t AS5x4x::SPIWriteChain(AS5x4x *device, uint16_t numchain, uint8_t *wtdata) {
/**************************************************************************************************
 * Externally callable function, which is able to cycle through the AS5x4x device(s) in the daisy
 * chain configuration - The the devices datasheet for how to configure a daisy chain.
 *
 * This function will receive a pointer to a list of defined AS5x4x classes, along with the number
 * of devices in the chain. The final argument is a pointer to the SPI array (8bits) which will
 * contain the data to be transmitted.
 *      >> NOTE <<
 *          Ordering of the AS5x4x pointer/array is important - the first entry in the array is
 *          the LAST device in the daisy chain.
 *
 * The function will then go through the internal write buffers of each of the linked AS5x4x
 * classes and see if there is anything to be transmitted.
 * If all of the devices are empty, then the function will exit. If at least one device has data
 * to be written, then the function will continue.
 * If data is to be transmitted, but some of the devices are not requesting any data, then this
 * function will force a "NOP" to be requested.
 *      This is required such that the "ReadDataPacket" works correctly, as relies upon the
 *      entries of the internal read and write buffers to be in sync.
 *
 * Function will then return the number of SPI write (8bit) array entries used.
 *************************************************************************************************/
    uint16_t packetdata = 0;        // Variable to store the packet to transmit
    uint8_t getmeout = 1;           // Variable to break out of function if there is no data to
                                    // transmit (initialise to "break out" = 1)
    AS5x4x *startposition = device; // Retain the start position

    uint16_t i = 0;                 // Variable to loop through the devices attached in the chain
            // note, the first entry in the list, is the LAST device in the chain

    for (i = 0; i != numchain; i++) {                       // Loop through the AS5x4x pointer
        if (device->wtBuff->State() != GenBuffer_Empty) {   // If there is data to be transmitted
            getmeout = 0;                                   // Set get out variable to "0", do not
                                                            // exit function
            break;                                          // Then break out of loop
        }
        device++;                                           // Increment the AS5x4x pointer by 1
    }

    if (getmeout == 1)          // If the get out variable to still "1", i.e. no data found to be
        return(0);              // transmitted, then exit function - returning a value of "0"

    device = startposition;     // Re-link the "device" pointer to start

    uint16_t arraysize = 0;     // Variable to store the number of SPI packets to transmit

    for (i = 0; i != numchain; i++) {                       // Loop through the AS5x4x pointer
        // Then read the contents of the AS5x4x write buffer.
        if (device->wtBuff->State() == GenBuffer_Empty) {   // If the AS5x4x device is empty
            device->constructNOP();                         // Force a NOP request
            // This will ensure that the "ReadDataPacket" will still function correctly
        }

        device->wtBuff->OutputRead(&packetdata);    // Read from internal buffer (no need to check
                                                    // state, as already checked

        *wtdata = (uint8_t) (packetdata >> 8);      // Take the MSByte of the packet to transmit
                                                    // and put into SPI array
        wtdata++; arraysize++;                      // increment SPI array pointer, and array size

        *wtdata = (uint8_t) (packetdata);           // Take the LSByte of the packet to transmit
                                                    // and put into SPI array
        wtdata++; arraysize++;                      // increment SPI array pointer, and array size

        device++;                                   // Increment the AS5x4x device pointer
    }

    return (arraysize);     // Once complete return the number of SPI array entries populated
}

uint8_t AS5x4x::SPIReadChain(AS5x4x *device, uint16_t numchain, uint8_t *rddata, uint16_t size) {
/**************************************************************************************************
 * Externally callable function, which is able to cycle through the input SPI read data, read from
 * AS5x4x device(s) in the daisy chain configuration -  The the devices datasheet for how to
 * configure a daisy chain.
 *
 * This function will receive a pointer to a list of defined AS5x4x classes, along with the number
 * of devices in the chain. Along with the SPI read array, and the size of entries read from the
 * SPI peripheral.
 *      >> NOTE <<
 *          Ordering of the AS5x4x pointer/array is important - the first entry in the array is
 *          the LAST device in the daisy chain.
 *
 * The function will then go through the SPI read data, combine together 2 entries to get a single
 * packet read from the AS5x4x device (16bits). This is then added to the internal read buffer of
 * each of the AS5x4x devices in the input pointer.
 * Throughout the number of entries read is compared to the input SPI read size, to ensure that
 * the correct number of entries have been provided. If this is incorrect function will return
 * "-1".
 * Also at the end of the function, the final number of array entries read is compared with this
 * value again, to ensure that they are equal - function has worked correctly, will therefore
 * return "0". Otherwise a fault has occurred = "-1".
 *************************************************************************************************/
    uint8_t MSB = 0, LSB = 0;   // Variables to store the Upper (MSB) and Lower (LSB) byte of data
    uint16_t arraysize = 0;     // Variable to store the number of SPI array entries read
                                // -> To be compared with input "size" to ensure no overflow.

    uint16_t i = 0;             // Variable to loop through the devices attached in the chain and
                                // link the read data to internal buffers

    if (size == 0)              // If the input size is zero (i.e. nothing in array)
        return(-1);             // return "-1"

    for (i = 0; i != numchain; i++) {
        MSB = *rddata;          // The first entry will be the "MSB", so link to MSB variable
        rddata++; arraysize++;  // Increment SPI array entry, and calculated array size

        if (arraysize > size) return(-1);   // If the calculated array size, exceeds input "size"
                                            // then exit, and return "-1"

        LSB = *rddata;          // The second entry will be the "LSB", so link to LSB variable
        rddata++; arraysize++;  // Increment SPI array entry, and calculated array size

        if (arraysize > size) return(-1);   // If the calculated array size, exceeds input "size"
                                            // then exit, and return "-1"
        // Can now populate the data into the AS5x4x internal read buffer
        device->rdBuff->InputWrite((uint16_t) ((MSB << 8) | LSB));  // Combine together MSB and LSB
                                                                    // and put into buffer
        device++;               // Then increment the AS5x4x device pointer
    }

    if (arraysize == size)      // If the calculated array size, equals input size
        return(0);              // then function has worked correctly - return "0"
    else                        // OTHERWISE
        return(-1);             // function has not worked correctly, or input wrong - return "-1"

}

void AS5x4x::deconstructAS5048A(uint16_t Address, uint16_t packetdata) {
/**************************************************************************************************
 * Function will be called within the "ReadDataPacket" function, if the AS5x4x device has been
 * constructed as a "AS5048A" device.
 * Function will then read the input requested "Address" - read from the write buffer, and the
 * "packetdata" read from the device.
 * Depending upon the address how to understand the "packetdata" will vary.
 * Consult the AS5048A datasheet to see how this has been constructed.
 *************************************************************************************************/
    if      (Address == AS5048_ERRFL) {     // If ERRFL then
        //=========================================================================================
        packetdata  &= AS5x4x_ErrorMask;            // Retrieve only the error bits

        this->Flt = AS5x4x_NoFault;                 // Default the data as being "No Fault"
            // This will then be overridden if there is data in the register

        if      (packetdata == AS5x4x_FrameError)   // If the error equals "Frame Error"
            this->Flt = AS5x4x_Frame;               // Set the fault state to "AS5x4x_Frame"

        else if (packetdata == AS5x4x_CmdIvError)   // If the error equals "Command Invalid"
            this->Flt = AS5x4x_CmdInv;              // Set the fault state to "AS5x4x_CmdInv"

        else if (packetdata == AS5x4x_ParitError)   // If the error equals "Parity Fault"
            this->Flt = AS5x4x_Parity;              // Set the fault state to "AS5x4x_Parity"
    }
    else if (Address == AS5048_DIAAGC) {    // If DIAAGC then
        //=========================================================================================
        this->AGC           = (packetdata & AS5x4x_AGVValueMask);       // Retrieve the AGC Values
        this->Diagnostic    = ((packetdata & AS5x4x_DiagnFlags) >> 8);  // Retrieve the Diagnostic
                                                                        // flags and shift down
    }
    else if (Address == AS5048_MAG) {       // If MAG then
        //=========================================================================================
        this->Mag           = (packetdata & AS5x4x_DataMask);       // Retrieve the Magnitude value
    }
    else if (Address == AS5048_ANGLE) {     // If ANGLE then
        //=========================================================================================
        this->AngularSteps  = (packetdata & AS5x4x_DataMask);       // Retrieve the Angle value
        this->Angle         = (((float)this->AngularSteps) * PI * 2) / 16383;
            // Convert to radians
    }
    else {}
}

void AS5x4x::deconstructAS5047D(uint16_t Address, uint16_t packetdata) {
/**************************************************************************************************
 * Function will be called within the "ReadDataPacket" function, if the AS5x4x device has been
 * constructed as a "AS5047D" device.
 * Function will then read the input requested "Address" - read from the write buffer, and the
 * "packetdata" read from the device.
 * Depending upon the address how to understand the "packetdata" will vary.
 * Consult the AS5047D datasheet to see how this has been constructed.
 *************************************************************************************************/
    if      (Address == AS5047_ERRFL) {     // If ERRFL then
        //=========================================================================================
        packetdata  &= AS5x4x_ErrorMask;            // Retrieve only the error bits

        this->Flt = AS5x4x_NoFault;                 // Default the data as being "No Fault"
            // This will then be overridden if there is data in the register

        if      (packetdata == AS5x4x_FrameError)   // If the error equals "Frame Error"
            this->Flt = AS5x4x_Frame;               // Set the fault state to "AS5x4x_Frame"

        else if (packetdata == AS5x4x_CmdIvError)   // If the error equals "Command Invalid"
            this->Flt = AS5x4x_CmdInv;              // Set the fault state to "AS5x4x_CmdInv"

        else if (packetdata == AS5x4x_ParitError)   // If the error equals "Parity Fault"
            this->Flt = AS5x4x_Parity;              // Set the fault state to "AS5x4x_Parity"
    }
    else if (Address == AS5047_DIAAGC) {    // If DIAAGC then
        //=========================================================================================
        this->AGC           = (packetdata & AS5x4x_AGVValueMask);       // Retrieve the AGC Values
        this->Diagnostic    = ((packetdata & AS5x4x_DiagnFlags) >> 8);  // Retrieve the Diagnostic
                                                                        // flags and shift down
    }
    else if (Address == AS5047_MAG) {       // If MAG then
        //=========================================================================================
        this->Mag           = (packetdata & AS5x4x_DataMask);       // Retrieve the Magnitude value
    }
    else if (Address == AS5047_ANGLE) {     // If ANGLE then
        //=========================================================================================
        this->AngularSteps  = (packetdata & AS5x4x_DataMask);       // Retrieve the Angle value
        this->Angle         = (((float)this->AngularSteps) * PI * 2) / 16383;
            // Convert to radians
    }
    else {}
}

void AS5x4x::constructNOP(void) {
/**************************************************************************************************
 * Function will construct a request to the AS5x4x device of "NOP" = No Operation.
 * Will vary the address depending upon how the AS5x4x class has been constructed.
 *************************************************************************************************/
    if (this->Device == AS5048A)                                // If device is AS5048A
        this->WriteDataPacket(AS5048_NOP);                      // Transmit the NOP

    else if (this->Device == AS5047D)                           // If device is AS5047D
        this->WriteDataPacket(AS5047_NOP);                      // Transmit the NOP
}

void AS5x4x::constructCEF(void) {
/**************************************************************************************************
 * Function will construct a request to the AS5x4x device for "CEF" = Clear Error Flags.
 * Will vary the address depending upon how the AS5x4x class has been constructed.
 *************************************************************************************************/
    if (this->Device == AS5048A)                                // If device is AS5048A
        this->WriteDataPacket(AS5048_ERRFL | AS5x4x_ReadMask);  // Transmit the Read request for
                                                                // reading the ERRFL address

    else if (this->Device == AS5047D)                           // If device is AS5047D
        this->WriteDataPacket(AS5047_ERRFL | AS5x4x_ReadMask);  // Transmit the Read request for
                                                                // reading the ERRFL address
}


void AS5x4x::constructAGC(void) {
/**************************************************************************************************
 * Function will construct a request to the AS5x4x device for "AGC" = Automatic Gain Control.
 * Will vary the address depending upon how the AS5x4x class has been constructed.
 *************************************************************************************************/
    if (this->Device == AS5048A)                                // If device is AS5048A
        this->WriteDataPacket(AS5048_DIAAGC | AS5x4x_ReadMask); // Transmit the Read request for
                                                                // reading the DIAAGC address

    else if (this->Device == AS5047D)                           // If device is AS5047D
        this->WriteDataPacket(AS5047_DIAAGC | AS5x4x_ReadMask); // Transmit the Read request for
                                                                // reading the DIAAGC address
}

void AS5x4x::constructMag(void) {
/**************************************************************************************************
 * Function will construct a request to the AS5x4x device for "MAG" = Magnitude from CORDIC
 * Will vary the address depending upon how the AS5x4x class has been constructed.
 *************************************************************************************************/
    if (this->Device == AS5048A)                                // If device is AS5048A
        this->WriteDataPacket(AS5048_MAG | AS5x4x_ReadMask);    // Transmit the Read request for
                                                                // reading the Magnitude address

    else if (this->Device == AS5047D)                           // If device is AS5047D
        this->WriteDataPacket(AS5047_MAG | AS5x4x_ReadMask);    // Transmit the Read request for
                                                                // reading the Magnitude address
}

void AS5x4x::constructAng(void) {
/**************************************************************************************************
 * Function will construct a request to the AS5x4x device for "ANG" = Angle
 * Will vary the address depending upon how the AS5x4x class has been constructed.
 *************************************************************************************************/
    if (this->Device == AS5048A)                                // If device is AS5048A
        this->WriteDataPacket(AS5048_ANGLE | AS5x4x_ReadMask);  // Transmit the Read request for
                                                                // reading the Angle address

    else if (this->Device == AS5047D)                           // If device is AS5047D
        this->WriteDataPacket(AS5047_ANGLE | AS5x4x_ReadMask);  // Transmit the Read request for
                                                                // reading the Angle address
}

void AS5x4x::constructALL(void) {
/**************************************************************************************************
 * Function simply goes through all of the construct function, to request a full update of all
 * readable registers of the AS5x4x device.
 * Not that the final entry is a "NOP", this is to ensure that the ANG response is retrieved
 * within the same request.
 *************************************************************************************************/
    this->constructCEF();
    this->constructAGC();
    this->constructMag();
    this->constructAng();
    this->constructNOP();

}

void AS5x4x::DirectSPITransmit(SPIDevice *Interface, GPIO *CS) {
/**************************************************************************************************
 * Function will go through all of the internal write buffer contents of "this" AS5x4x device, and
 * construct a SPI write 8bit array. Which will then be written to the SPI device provided as
 * input to this function.
 *      >> NOTE <<
 *      A Chip Select GPIO has to be provided, as software control of the SPI CS is required to
 *      use the AS5x4x attached device correctly.
 *
 * Function will also populate the internal read buffer of "this" As5x4x device.
 *************************************************************************************************/
    uint8_t writedata[2]= { 0 };            // Array to store the SPI write entries
    uint8_t readdata[2] = { 0 };            // Array to store the SPI read entries

    while(this->wtBuff->State() != GenBuffer_Empty) {       // Keep cycling until the internal
                                                            // write buffer is empty
        AS5x4x::SPIWriteChain(this, 1, writedata);          // Populate the SPI write array

        Interface->SPITransfer(CS, writedata, readdata, 2); // Transmit data via SPI

        AS5x4x::SPIReadChain(this, 1, readdata, 2);         // Put the SPI read array into internal
                                                            // read buffer
    }
}

AS5x4x::~AS5x4x() {
/**************************************************************************************************
 * When the destructor is called, need to ensure that the memory allocation is cleaned up, so as
 * to avoid "memory leakage"
 *************************************************************************************************/
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#ifdef __LiteImplement__        // If "__LiteImplement__" has been defined, then need to have array
                                // fully defined, and provided to the "GenBuffer"
                                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#else                           // If "__LiteImplement__" has not been defined, then allow use of
                                // "new" and "delete" for defining internal arrays
                                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    delete [] rdBuff;           // Delete the array for "rdBuff"
    delete [] wtBuff;           // Delete the array for "wtBuff"

#endif                          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}
