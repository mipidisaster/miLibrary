/**************************************************************************************************
 * @file        AS5x4x.cpp
 * @author      Thomas
 * @version     V1.1
 * @date        21 Dec 2018
 * @brief       Source file for the AMS Angular Position device (AS5x4x)
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include "FileIndex.h"
#include FilInd_AS5x4x_HD

void AS5x4x::popGenParam(void) {
/**************************************************************************************************
 * Generate default parameters for the AS5x4x device. To be called by all constructors.
 * Initial construction will populate the internal GenBuffers with default parameters (as basic
 * constructor of GenBuffer is already set to zero).
 *************************************************************************************************/
    this->Angle         = -999;                 // All parameters set to zero
    this->AngularSteps  = 0;                    //
    this->Mag           = 0;                    //
    this->AGC           = 0;                    //

    this->Flt           = DevFlt::None;         // Set fault state to initialised
    this->Diagnostic    = 0;                    // Initialise Diagnostic flags to zero
    this->Device        = DevPart::AS5048A;     // Default is "AS5048A"
}

AS5x4x::AS5x4x(void) {
/**************************************************************************************************
 * Basic construction of AS5x4x Device
 *************************************************************************************************/
    this->popGenParam();                        // Populate generic class parameters
}

void AS5x4x::create(DevPart Device, uint16_t *wtBuff, uint16_t *rdBuff, uint32_t size) {
/**************************************************************************************************
 * Create a AS5x4x class. For construction the of the class, the Device type needs to be provided,
 * along with a pointer to the Write and Read buffers, which will be used internally to the class
 * to manage the data that is transfered to the attached device.
 * The class construction will initialise all the parameters, and set the fault state to
 * initialised.
 *************************************************************************************************/
    this->popGenParam();                        // Populate generic class parameters
    this->Device        = Device;               // Store the device

    // Setup buffer pointers:
    this->wtBuff.create(wtBuff, size);          // Create internal GenBuffer
    this->rdBuff.create(rdBuff, size);          // Create internal GenBuffer


    this->wtBuff.QFlush();                      // Flush data
    this->rdBuff.QFlush();                      // Flush data
}

AS5x4x::AS5x4x(DevPart Device, uint16_t *wtBuff, uint16_t *rdBuff, uint32_t size) {
/**************************************************************************************************
 * Create a AS5x4x class. For construction the of the class, the Device type needs to be provided,
 * along with a pointer to the Write and Read buffers, which will be used internally to the class
 * to manage the data that is transfered to the attached device.
 * The class construction will initialise all the parameters, and set the fault state to
 * initialised.
 *************************************************************************************************/
    this->create(Device, wtBuff, rdBuff, size);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#ifndef __LiteImplement__       // If "__LiteImplement__" has not been defined, then allow use of
                                // "new" and "delete" for defining internal arrays
                                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
AS5x4x::AS5x4x(_AS5x4xDev Device, uint32_t Buffersize) {
/**************************************************************************************************
 * Create a AS5x4x class. For construction the of the class, the Device type needs to be provided.
 * The class construction will initialise all the parameters, and set the fault state to
 * initialised.
 *************************************************************************************************/
    this->popGenParam();                        // Populate generic class parameters
    this->Device        = Device;               // Store the device

    // Setup buffer pointers:
    this->wtBuff        = new GenBuffer<uint16_t>(Buffersize);
    this->rdBuff        = new GenBuffer<uint16_t>(Buffersize);
}
#endif                          //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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

void AS5x4x::WriteDataPacket(uint16_t PacketData) {
/**************************************************************************************************
 * Function will take the input packet to be transmitted to the AS5x4x device.
 * Prior to putting into the internal write buffer, it will be checked to ensure the data is of
 * EVEN parity.
 *************************************************************************************************/
    if (this->EvenParityCheck(PacketData) != 1)         // If not EVEN parity then
        PacketData |= AS5x4x_PARITY;                    // make data EVEN parity

    this->wtBuff.InputWrite(PacketData);                // Put data into the buffer
}

uint16_t AS5x4x::SPIWriteChain(AS5x4x *device, uint16_t numchain, GenBuffer<uint8_t> *wtdata) {
/**************************************************************************************************
 * Externally callable function, which is able to cycle through the AS5x4x device(s) in the daisy
 * chain configuration - See the devices datasheet for how to configure a daisy chain.
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
    uint16_t i = 0;                 // Variable to loop through the devices attached in the chain
            // note, the first entry in the list, is the LAST device in the chain

    for (i = 0; i != numchain; i++) {                       // Loop through the AS5x4x pointer
        if (device[i].wtBuff.State() != GenBuffer_Empty) {  // If there is data to be transmitted
            getmeout = 0;                                   // Set get out variable to "0", do not
                                                            // exit function
            break;                                          // Then break out of loop
        }
    }

    if (getmeout == 1)          // If the get out variable to still "1", i.e. no data found to be
        return(0);              // transmitted, then exit function - returning a value of "0"

    uint16_t arraysize = 0;     // Variable to store the number of SPI packets to transmit

    for (i = 0; i != numchain; i++) {                       // Loop through the AS5x4x pointer
        // Then read the contents of the AS5x4x write buffer.
        if (device[i].wtBuff.State() == GenBuffer_Empty) {  // If the AS5x4x device is empty
            device[i].constructNOP();                       // Force a NOP request
            // This will ensure that the "ReadDataPacket" will still function correctly
        }

        device[i].wtBuff.OutputRead(&packetdata);   // Read from internal buffer (no need to check
                                                    // state, as already checked

        wtdata->InputWrite(  (uint8_t) (packetdata >> 8)  );    // Take the MSByte of the packet
                                                                // to transmit and put into SPI
                                                                // buffer.
        arraysize++;                                            // increment array size

        wtdata->InputWrite(  (uint8_t) (packetdata)  );         // Take the LSByte of the packet
                                                                // to transmit and put into SPI
                                                                // buffer.
        arraysize++;                                            // increment array size
    }

    return (arraysize);     // Once complete return the number of SPI array entries populated
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

        this->Flt = DevFlt::None;                   // Default the data as being "No Fault"
            // This will then be overridden if there is data in the register

        if      (packetdata == AS5x4x_FrameError)   // If the error equals "Frame Error"
            this->Flt = DevFlt::Frame;              // Set the fault state to "AS5x4x_Frame"

        else if (packetdata == AS5x4x_CmdIvError)   // If the error equals "Command Invalid"
            this->Flt = DevFlt::CmdInv;             // Set the fault state to "AS5x4x_CmdInv"

        else if (packetdata == AS5x4x_ParitError)   // If the error equals "Parity Fault"
            this->Flt = DevFlt::Parity;             // Set the fault state to "AS5x4x_Parity"

        else if (packetdata != 0x0000)              // If register is none zero
            this->Flt = DevFlt::MultiFault;         // Then multiple faults have occured
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

        this->Flt = DevFlt::None;                   // Default the data as being "No Fault"
            // This will then be overridden if there is data in the register

        if      (packetdata == AS5x4x_FrameError)   // If the error equals "Frame Error"
            this->Flt = DevFlt::Frame;              // Set the fault state to "AS5x4x_Frame"

        else if (packetdata == AS5x4x_CmdIvError)   // If the error equals "Command Invalid"
            this->Flt = DevFlt::CmdInv;             // Set the fault state to "AS5x4x_CmdInv"

        else if (packetdata == AS5x4x_ParitError)   // If the error equals "Parity Fault"
            this->Flt = DevFlt::Parity;              // Set the fault state to "AS5x4x_Parity"

        else if (packetdata != 0x0000)              // If register is none zero
            this->Flt = DevFlt::MultiFault;         // Then multiple faults have occured
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

uint8_t AS5x4x::SPIReadChain(AS5x4x *device, uint16_t numchain,
                             GenBuffer<uint8_t> *rddata, uint16_t size) {
/**************************************************************************************************
 * Externally callable function, which is able to cycle through the input SPI read data, read from
 * AS5x4x device(s) in the daisy chain configuration -  See the devices datasheet for how to
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

    if ( ( size % (numchain * 2) ) != 0 )   // If input size is not a multiple of devices in chain
                                            // Multiplied by 2
        return(-1);                         // Then there is a request fault

    while(arraysize != size) {
        for (i = 0; i != numchain; i++) {
            rddata->OutputRead(&MSB);   // The first entry will be the "MSB", so link to MSB
                                        // variable
            arraysize++;                // Increment calculated array size

            rddata->OutputRead(&LSB);   // The first entry will be the "LSB", so link to MSB
                                        // variable
            arraysize++;                // Increment calculated array size
            // Can now populate the data into the AS5x4x internal read buffer
            device[i].rdBuff.InputWrite((uint16_t) ((MSB << 8) | LSB)); // Combine together MSB,
                                                                        // LSB and put into buffer
    } }

    return (0);     // Return "0"
}

void AS5x4x::constructNOP(void) {
/**************************************************************************************************
 * Function will construct a request to the AS5x4x device of "NOP" = No Operation.
 * Will vary the address depending upon how the AS5x4x class has been constructed.
 *************************************************************************************************/
    if (this->Device == DevPart::AS5048A)                       // If device is AS5048A
        this->WriteDataPacket(AS5048_NOP);                      // Transmit the NOP

    else if (this->Device == DevPart::AS5047D)                  // If device is AS5047D
        this->WriteDataPacket(AS5047_NOP);                      // Transmit the NOP
}

void AS5x4x::constructCEF(void) {
/**************************************************************************************************
 * Function will construct a request to the AS5x4x device for "CEF" = Clear Error Flags.
 * Will vary the address depending upon how the AS5x4x class has been constructed.
 *************************************************************************************************/
    if (this->Device == DevPart::AS5048A)                       // If device is AS5048A
        this->WriteDataPacket(AS5048_ERRFL | AS5x4x_ReadMask);  // Transmit the Read request for
                                                                // reading the ERRFL address

    else if (this->Device == DevPart::AS5047D)                  // If device is AS5047D
        this->WriteDataPacket(AS5047_ERRFL | AS5x4x_ReadMask);  // Transmit the Read request for
                                                                // reading the ERRFL address
}

void AS5x4x::constructAGC(void) {
/**************************************************************************************************
 * Function will construct a request to the AS5x4x device for "AGC" = Automatic Gain Control.
 * Will vary the address depending upon how the AS5x4x class has been constructed.
 *************************************************************************************************/
    if (this->Device == DevPart::AS5048A)                       // If device is AS5048A
        this->WriteDataPacket(AS5048_DIAAGC | AS5x4x_ReadMask); // Transmit the Read request for
                                                                // reading the DIAAGC address

    else if (this->Device == DevPart::AS5047D)                  // If device is AS5047D
        this->WriteDataPacket(AS5047_DIAAGC | AS5x4x_ReadMask); // Transmit the Read request for
                                                                // reading the DIAAGC address
}

void AS5x4x::constructMag(void) {
/**************************************************************************************************
 * Function will construct a request to the AS5x4x device for "MAG" = Magnitude from CORDIC
 * Will vary the address depending upon how the AS5x4x class has been constructed.
 *************************************************************************************************/
    if (this->Device == DevPart::AS5048A)                       // If device is AS5048A
        this->WriteDataPacket(AS5048_MAG | AS5x4x_ReadMask);    // Transmit the Read request for
                                                                // reading the Magnitude address

    else if (this->Device == DevPart::AS5047D)                  // If device is AS5047D
        this->WriteDataPacket(AS5047_MAG | AS5x4x_ReadMask);    // Transmit the Read request for
                                                                // reading the Magnitude address
}

void AS5x4x::constructAng(void) {
/**************************************************************************************************
 * Function will construct a request to the AS5x4x device for "ANG" = Angle
 * Will vary the address depending upon how the AS5x4x class has been constructed.
 *************************************************************************************************/
    if (this->Device == DevPart::AS5048A)                       // If device is AS5048A
        this->WriteDataPacket(AS5048_ANGLE | AS5x4x_ReadMask);  // Transmit the Read request for
                                                                // reading the Angle address

    else if (this->Device == DevPart::AS5047D)                  // If device is AS5047D
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

uint8_t AS5x4x::checkDataRequest(void) {
/**************************************************************************************************
 * Check device to see if there is any new data requests available
 *************************************************************************************************/
    if (this->wtBuff.State() != GenBuffer_Empty )
        return (1);
    else
        return (0);
}

void AS5x4x::readDataPacket(void) {
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

    _GenBufState ExitLoop = this->rdBuff.State();       // Get current state of buffer

    while(ExitLoop != GenBuffer_Empty) {
        // Retrieve the position of the current readback, and then loop back a previous entry
        if (this->rdBuff.output_pointer == 0)           // If start of buffer
            sentpacket = this->rdBuff.length - 1;       // Look at end of buffer
        else                                            // Otherwise loop back
            sentpacket = (this->rdBuff.output_pointer - 1) % this->rdBuff.length;

        tempreq = this->wtBuff.pa[sentpacket];          // Retrieve the requested data
        ExitLoop = this->rdBuff.OutputRead(&tempread);  // Read back the Read data (and determine
                                                        // state of buffer)

        if (this->EvenParityCheck(tempread) != 1)       // If the read data is not EVEN parity
            this->Flt = DevFlt::Parity;                 // Set the fault to "Parity" fault

        else {                                          // If the read data is EVENT then
            tempreq &= AS5x4x_DataMask;                 // Only get the address data
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            if (this->Device == DevPart::AS5048A) {             // If device is AS5048A
                this->deconstructAS5048A(tempreq, tempread);    // Call AS5048A deconstruction
            }
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            else if (this->Device == DevPart::AS5047D) {        // If device is AS5047D
                this->deconstructAS5047D(tempreq, tempread);    // Call AS5047D deconstruction
            }
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        }
    }
}

AS5x4x::Daisy AS5x4x::constructDaisy(AS5x4x *device, uint16_t numchain) {
/**************************************************************************************************
 * Construct the Daisy Chain structure to hold the AS5x4x device array, and size number.
 *************************************************************************************************/
    AS5x4x::Daisy newchain = { 0 };             // Generate the "AS5x4x::Daisy" structure with
                                                // default values of zero.

    newchain.Devices    = device;               // Link input pointer to "Daisy"
    newchain.numDevices = numchain;             // Put chain size into "Daisy"

    newchain.Flt    = SPIPeriph::DevFlt::None;  // Default all status indicators to "0"/"None"
    newchain.Trgt   = 0;                        //
    newchain.Cmplt  = 0;                        //

    return (  newchain  );      // Return chain
}

uint8_t AS5x4x::checkDaisyRequest(AS5x4x::Daisy *chain) {
/**************************************************************************************************
 * Check all of the devices in the chain, and see if there is any new requests in any of them
 *************************************************************************************************/
    uint16_t i = 0;         // Clear loop variable

    for (i = 0; i != chain->numDevices; i++) {                      // Loop through devices
        if (chain->Devices[i].checkDataRequest() == 1) {            // If data is present
            return (1);                 // Return (1) if there is some data in any device on chain
    } }

    // If made it to this point, then no data is in any of the devices on the chain
    return (0);
}

void AS5x4x::readDaisyPackets(AS5x4x::Daisy *chain) {
/**************************************************************************************************
 * Loop through all the devices within the Daisy chain, and read any of the stored packets
 *************************************************************************************************/
    uint16_t i = 0;

    for (i = 0; i != chain->numDevices; i++) {
        chain->Devices[i].readDataPacket();
    }
}

void AS5x4x::poleSPITransmit(SPIPeriph *Interface, GPIO *CS) {
/**************************************************************************************************
 * Function will go through all of the internal write buffer contents of "this" AS5x4x device, and
 * construct a SPI write 8bit array. Which will then be written to the SPI device provided as
 * input to this function.
 *      >> NOTE <<
 *      A Chip Select GPIO has to be provided, as software control of the SPI CS is required to
 *      use the AS5x4x attached device correctly.
 *
 * Function will also populate the internal read buffer of "this" As5x4x device. So if the device
 * is connected to other AS5x4x devices in a daisy chain, this function will not yield correct
 * data.
 *************************************************************************************************/
    uint8_t writedata[3]= { 0 };            // Array to store the SPI write entries
    uint8_t readdata[3] = { 0 };            // Array to store the SPI read entries

    GenBuffer<uint8_t> wBuff(&writedata[0], 3);      // Convert into temporary buffer
    GenBuffer<uint8_t> rBuff(&readdata[0], 3);       // Convert into temporary buffer


    while(this->checkDataRequest() != 0) {          // Keep cycling until the internal
                                                            // write buffer is empty
        AS5x4x::SPIWriteChain(this, 1, &wBuff);             // Populate the SPI write array

        Interface->poleMasterTransfer(CS, writedata, readdata, 2);  // Transmit data via SPI

        rBuff.input_pointer = 2;    // Update read buffer to contain 2 entries

        AS5x4x::SPIReadChain(this, 1, &rBuff, 2);           // Put the SPI read array into internal
                                                            // read buffer

        rBuff.QFlush();    // Flush all data in read buffer
        wBuff.QFlush();    // Flush all data in write buffer
    }

    this->readDataPacket();         // Read all data packets
}

void AS5x4x::reInitialise(void) {
/**************************************************************************************************
 * This function is to be called when the internal mechanics of the class need to be reset. This
 * is likely to occur in fault situations, or at initialisation.
 *************************************************************************************************/
    this->wtBuff.QFlush();                  // Clear the internal write buffer
    this->rdBuff.QFlush();                  // Clear the internal read buffer

    this->Flt           = DevFlt::Initialised;      // Set fault to initialised
}

void AS5x4x::poleSPITransmit(AS5x4x::Daisy *chain, SPIPeriph *Interface, GPIO *CS) {
/**************************************************************************************************
 * Function will go through all of the internal write buffer contents of "this" AS5x4x device, and
 * construct a SPI write 8bit array. Which will then be written to the SPI device provided as
 * input to this function.
 *      >> NOTE <<
 *      A Chip Select GPIO has to be provided, as software control of the SPI CS is required to
 *      use the AS5x4x attached device correctly.
 *
 * Function will also populate the internal read buffer of "this" As5x4x device. So if the device
 * is connected to other AS5x4x devices in a daisy chain, this function will not yield correct
 * data.
 *
 * >>> OVERLOADED FUNCTION <<<
 *   This version reads in the "Daisy" structure so as to communication with multiple devices
 *   in the chain
 *************************************************************************************************/
    uint16_t packetsize = 0;                // Variable to store number of bytes to transmit

    uint8_t writedata[(AS5x4x_MAXChain * 2) + 1]= { 0 };// Array to store the SPI write entries
    uint8_t readdata[(AS5x4x_MAXChain * 2) + 1] = { 0 };// Array to store the SPI read entries

    GenBuffer<uint8_t> wBuff(&writedata[0], (AS5x4x_MAXChain * 2) + 1);     // Convert into temp
    GenBuffer<uint8_t> rBuff(&readdata[0],  (AS5x4x_MAXChain * 2) + 1);     // buffer

    while(AS5x4x::checkDaisyRequest(chain) != 0) {      // Keep cycling till all devices in chain
                                                        // have no new requests to transmit
        packetsize = AS5x4x::SPIWriteChain(chain->Devices, chain->numDevices, &wBuff);
            // Populate the SPI write array

        Interface->poleMasterTransfer(CS, &writedata[0], &readdata[0], packetsize);
            // Transmit data via SPI

        rBuff.input_pointer = packetsize;   // Update read buffer to contain number of entries
                                            // transmitted.
        AS5x4x::SPIReadChain(chain->Devices, chain->numDevices, &rBuff, packetsize);
            // Put the SPI read array into internal read buffer(s) of devices in chain

        rBuff.QFlush();    // Flush all data in read buffer
        wBuff.QFlush();    // Flush all data in write buffer
    }

    AS5x4x::readDaisyPackets(chain);        // Read all data packets
}

void AS5x4x::intSingleTransmit(SPIPeriph *hal_SPI, GPIO *CS,
                               GenBuffer<uint8_t> *rBuff, GenBuffer<uint8_t> *wBuff,
                               volatile SPIPeriph::DevFlt *fltReturn, volatile uint8_t *cmpFlag,
                               uint8_t *cmpTarget) {
/**************************************************************************************************
 * Interrupt based request for transmission of data to the "own" AS5x4x device.
 *
 * Done by building a SPI request form, and putting onto the device queue.
*************************************************************************************************/
    uint16_t numbytes = 0;          // Number of bytes to be transmitted
    while(this->checkDataRequest() != 0) {                  // Keep cycling until the internal
                                                            // write buffer is empty
        numbytes = AS5x4x::SPIWriteChain(this, 1, wBuff);  // Populate the GenBuffer

        // Now build the SPI Request Form:
        hal_SPI->intMasterTransfer(CS, numbytes,
                                   wBuff, rBuff,
                                   fltReturn, cmpFlag);

        *cmpTarget += numbytes;     // Update target count
    }
}

void AS5x4x::intSingleTransmit(SPIPeriph *hal_SPI, GPIO *CS,  Daisy *chain,
                               GenBuffer<uint8_t> *rBuff, GenBuffer<uint8_t> *wBuff) {
/**************************************************************************************************
 * Interrupt based request for transmission of data to multiple devices in a specific "Daisy"
 * format
 *
 * Done by building a SPI request form, and putting onto the device queue.
*************************************************************************************************/
    uint16_t numbytes = 0;          // Number of bytes to be transmitted

    while(AS5x4x::checkDaisyRequest(chain) != 0) {      // Keep cycling till all devices in chain
                                                        // have no new requests to transmit

        numbytes = AS5x4x::SPIWriteChain(chain->Devices, chain->numDevices, wBuff);

        hal_SPI->intMasterTransfer(CS, numbytes,
                                   wBuff, rBuff,
                                   &(chain->Flt), &(chain->Cmplt));

        chain->Trgt += numbytes;    // Update target count
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
