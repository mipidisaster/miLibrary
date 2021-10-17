/**************************************************************************************************
 * @file        Dynamixel.cpp
 * @author      Thomas
 * @brief       << Manually Entered >>
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
#include "Dynamixel/Dynamixel.h"

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
Dynamixel::Dynamixel(UART_HandleTypeDef *UART_Handle, uint8_t *CommsBoardLoc, uint16_t size,
                     GenBuffer<uint8_t> *receivearray, GenBuffer<uint8_t> *transmitarray)
/**************************************************************************************************
 * Create Dynamixel UART class handler. This class is based upon the UARTDevice class, but adds
 * additional functions specific to communicating with the Dynamixel devices - XL-320 for example.
 *
 * The constructor will pass the input argument to the UARTDevice constructor, along with the
 * locations of the "GenBuffer" Receive and Transmit arrays. They will be used to constructor the
 * UARTDevice.
 * The normal array will be used to generate the Dynamixel specific entries
 *
 * After the UARTDevice constructor, the specific constructor for Dynamixel will be called, this
 * will initialise all the states and arrays specific for communicating via the Dynamixel protocol.
 *************************************************************************************************/
    : UARTDevice(UART_Handle, receivearray, transmitarray) {
            // Call UARTDevice constructor
                                                //  (STM3232Fx Configuration)
    this->State     = Dynm_Idle;                // Set initial state to "Idle"
    this->SrcState  = Dynm_Nothing;             // Set search state to "Nothing"

    this->CommsBoard = CommsBoardLoc;           // Link "CommsBoard" to input array

    this->Length   = size;                      // Setup "Length" variable as per input size
    this->watermark= size;                      // Set current watermark to input size - such that
                                                // "CleanBoard()" will initialise full array

    this->maxpoint = 0;                         // Set max point to "0"
    this->curpoint = 0;                         // Set current point to "0"

    this->CleanBoard();                         // Call CleanBoard function - will clear all
                                                // entries upto current value of "maxpoint"
    // Will also set "maxpoint" back to "0"

}

#elif (zz__MiEmbedType__zz == 10)       // If the target device is an Raspberry Pi then
//=================================================================================================
Dynamixel::Dynamixel(const char *deviceloc, int baud, uint16_t size,
                     GenBuffer<uint8_t> *receivearray, GenBuffer<uint8_t> *transmitarray)
/**************************************************************************************************
 * Create Dynamixel UART class handler. This class is based upon the UARTDevice class, but adds
 * additional functions specific to communicating with the Dynamixel devices - XL-320 for example.
 *
 * The constructor will pass the input arguments for device location and baudrate to the UARTDevice
 * constructor, along with addresses of the receive and tranmit arrays, They will be used to
 * constructor the UARTDevice.
 *
 * After the UARTDevice constructor, the specific constructor for Dynamixel will be called, this
 * will initialise all the states and arrays specific for communicating via the Dynamixel protocol.
 *************************************************************************************************/
    : UARTDevice(deviceloc, baud, receivearray, transmitarray) {    // Call UARTDevice constructor
                                                //  (Raspberry Pi Configuration)
    this->State     = Dynm_Idle;                // Set initial state to "Idle"
    this->SrcState  = Dynm_Nothing;             // Set search state to "Nothing"

    this->CommsBoard = CommsBoardLoc;           // Link "CommsBoard" to input array

    this->Length   = size;                      // Setup "Length" variable as per input size
    this->watermark= size;                      // Set current watermark to input size - such that
                                                // "CleanBoard()" will initialise full array

    this->maxpoint = 0;                         // Set max point to "0"
    this->curpoint = 0;                         // Set current point to "0"

    this->CleanBoard();                         // Call CleanBoard function - will clear all
                                                // entries upto current value of "maxpoint"
    // Will also set "maxpoint" back to "0"
}

#else
//=================================================================================================
#error "Unrecognised target device"

#endif

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void Dynamixel::CleanBoard(void) {
/**************************************************************************************************
 * Clears the data stored within "CommsBoard".
 * Will only clear the data which has been populated, i.e. upto the end pointer "maxpoint"
 * After clearing, returns "maxpoint" to start
 *************************************************************************************************/
    uint16_t i = 0;                         // Looper variable to cycle through entries
    for (i = 0; i != this->watermark; i++)  // Loop through the written data in "CommsBoard"
        this->CommsBoard[i] = 0x00;         // and set it to zero

    this->QCleanBoard();                    // Call Quick Clean, as this will only initial all
                                            // pointers to "zero", and initialise the "Search
                                            // State"
    this->watermark = 0;    // As have now cleared the data, reset watermark.
}

void Dynamixel::QCleanBoard(void) {
/**************************************************************************************************
 * Quick clean of the Board, simplily by setting all pointers to zero i.e. "CommsBoard" is empty
 *************************************************************************************************/
    this->curpoint  = 0;
    this->maxpoint  = 0;                    // Return all pointers back to zero
    this->SrcState  = Dynm_Nothing;
}

void Dynamixel::UpdateWaterMark(void) {
/**************************************************************************************************
 * Updates watermark, by comparing to the "maxpoint", and selecting the larger of the two.
 *************************************************************************************************/
    if (this->maxpoint > this->watermark) {
        this->watermark = this->maxpoint;
    }
}

_UARTDevFlt Dynamixel::RequstTransmission(uint8_t ID, uint8_t Instr,
                                          uint8_t *param, uint16_t size) {
/**************************************************************************************************
 * Function to put requested data into the "Transmit" buffer.
 * If the number of parameters to be transmitted (i.e. input "size") is greater than the size of
 * the "CommsBoard" - taking into account standard package layout - then an error will be returned
 * and the data not put onto the queue.
 *
 * As the "Transmit" does not transmit the data via UART (it is "CommsBoard" that is used instead)
 * the layout of the buffer does not need to conform with the Dynamixel protocol. So instead only
 * the variable data is put onto the buffer - Package ID ("ID"), Instruction ("Instr"), the size of
 * parameter ("size") and the parameter data ("param").
 * Size is stored in the same format as Dynamixel protocol - little endian.
 *************************************************************************************************/
    uint16_t i;             // Variable used to loop through input array

    if (size > ((this->Length) - 10)) {         // If the size of the data to be transmitted is
        // greater than the defined size of the array, taking into account the size of the Header
        // (4 bytes), Packet ID (1 byte), packet size (2 byte), instruction (1 byte), and CRC (2
        // bytes), totalling 10 bytes.
        return (this->Flt = UART_DataError);    // Return error
    }

    this->Transmit->InputWrite(ID);     // Add ID to the "Transmit" buffer
    this->Transmit->InputWrite(Instr);  // Add the instruction to the "Transmit" buffer
    this->Transmit->InputWrite((uint8_t) (size & 0x00FF));      // Add lower part of size to buffer
    this->Transmit->InputWrite((uint8_t)((size >> 8) & 0x00FF));// Add upper part of size to buffer

    for (i = 0; i != size; i++)
        this->Transmit->InputWrite(param[i]);       // Add data onto buffer

    this->PkgCode();                    // Call Package Code (Generation)
    return (this->Flt = UART_NoFault);  // If make it this far, then indicate no error with data
}

void Dynamixel::PkgCode(void) {
/**************************************************************************************************
 * Function will take the data requested via "Transmit" buffer, and populate the transmission as
 * per the Dynamixel protocol.
 * It will only do this if the "State" of the interface is set to "Idle" and there if data in the
 * buffer.
 *
 * If these conditions are met, then it will pull the Package ID, Instruction, parameters and size
 * from the buffer - no additional checks are required as this buffer is internal.
 * The standard layout of the Dynamixel protocal will then generated within "CommsBoard", and each
 * of the specific entries will be added.
 * "CommsBoard" will then be placed within the CRC calculator and the output added to the end.
 * Once complete, the "State" of the interface will be set to "Speaking" to lock out any updates
 * of "CommsBoard" until tranmission is complete.
 *************************************************************************************************/
    if ( (this->State != Dynm_Idle) || (this->Transmit->State() == GenBuffer_Empty) )
        // If the state of Dynamixel is only set to "Idle", and there is data to be transmitted
        // then start building the "CommsBoard".
        // This is to ensure that the "CommsBoard" is only written to by user is "Speaking" state
        // and written to by outside devices in "Listening" state
        return;

    this->CleanBoard();     // Clear the "CommsBoard"
    this->State = Dynm_Speaking;            // Set the state to "Speaking" to lock out any changes
                                            // to "CommsBoard"

    // Put in the GPIO switch if this is used to manage the single wire UART


    // Should get to this point if state is "Idle", so now, pull data from "Transmission" buffer
    // and generate package in "CommsBoard"
    uint8_t ID = 0;         // Variable to store the Packet ID from buffer
    uint8_t Instr = 0;      // Variable to store the Package Instruction from buffer
    uint8_t size_L = 0;     // Variable to store the Package data size (Lower)
    uint8_t size_H = 0;     // Variable to store the Package data size (Upper)
    uint16_t bigsize = 0;   // Variable to store the total size entries
    uint16_t temp = 0;      // Temp variable for looping and calculating the CRC

    this->Transmit->OutputRead(&ID);    // The ID is the first entry point stored in internal
                                        // buffer
    this->Transmit->OutputRead(&Instr); // The Instruction is the second entry point stored in
                                        // internal buffer
    this->Transmit->OutputRead(&size_L);// The size lower byte is the third entry stored
    this->Transmit->OutputRead(&size_H);// The size lower byte is the third entry stored

    bigsize = (uint16_t) ((size_H << 8) | size_L); // Put together size, to get all data on buffer
    temp = bigsize + 3; // The size stored in the buffer is only the number of data points to
                        // transmit, need to add Header/CRC/etc.

    /* Generate the Package Header entries*/
    this->CommsBoard[0] = 0xFF;         // Generate Headers
    this->CommsBoard[1] = 0xFF;         // Generate Headers
    this->CommsBoard[2] = 0xFD;         // Generate Headers
    this->CommsBoard[3] = 0x00;         // Put in Reserved entries

    /* User defined parameters into package format entries*/
    this->CommsBoard[4] = ID;           // Put the ID onto "Board"

    this->CommsBoard[5] = (uint8_t) (temp & 0x00FF);        // Put new size calc onto "Board" lowr
    this->CommsBoard[6] = (uint8_t) ((temp >> 8) & 0x00FF); // Put new size calc onto "Board" uppr

    this->CommsBoard[7] = Instr;        // Put the Instruction onto "Board"

    /* User defined data into package format entries*/
    for (temp = 0; temp != bigsize; temp++)                         // Loop through data
        this->Transmit->OutputRead(&(this->CommsBoard[8 + temp]));  // And put data onto "Board"

    temp = update_crc(0, this->CommsBoard, bigsize + 8);            // Calculate CRC

    this->CommsBoard[8+bigsize] = (uint8_t) (temp & 0x00FF);        // Add to "CommsBoard"
    this->CommsBoard[9+bigsize] = (uint8_t) ((temp >> 8) & 0x00FF); // Add to "CommsBoard"

    this->maxpoint = bigsize + 10;          // Update end pointer to complete size of package
    this->curpoint = 0;                     // Initialise the current pointer to 0

    this->UpdateWaterMark();                // Update the watermark
}

uint8_t Dynamixel::PkgDecode(uint8_t *ID, uint8_t *Instr, uint16_t *size) {
/**************************************************************************************************
 * Function will take the data read and stored onto "CommsBoard", then pull out the Packet ID,
 * Instruction, and size of parameter dat. These will all be provided to the input parameters "ID",
 * "Instr" and "size".
 * A check of the CRC will then be conducted, if this is faulty, no data is written to the
 * "Receive" buffer, and the UART interface is set as faulty.
 * If the CRC passes, then the parameter data is put into the "Receive" buffer to be read outside
 * of this function.
 *
 *      Outside function will need to read the parameter data stored in "Receive", otherwise the
 *      layout of next read will fail.
 *
 * The "CommsBoard" will then be cleared.
 *************************************************************************************************/
    if ( ( (this->State != Dynm_Idle) || (this->State != Dynm_Listening) ) &&
           (this->SrcState != Dynm_Complete) )
        // If "State" of the interface is not equal to either IDLE or Listening, and no complete
        // package has been read
        return(0);                                  // Then exit function

    // All Receiving functions will indicate if a complete data package has been found, and will
    // then set "SrcState" to "Complete", and "curpoint" will indicate the first package in data
    uint8_t size_L = 0;     // Variable to store the Package data size (Lower)
    uint8_t size_H = 0;     // Variable to store the Package data size (Upper)
    uint16_t bigsize = 0;   // Variable to store the total size entries
    uint16_t temp = 0;      // Temp variable for looping and calculating the CRC

    *ID     = this->CommsBoard[this->curpoint + 4];     // Read the Packet ID
    size_L  = this->CommsBoard[this->curpoint + 5];     // Read size (lower byte)
    size_H  = this->CommsBoard[this->curpoint + 6];     // Read size (upper byte)
    *Instr  = this->CommsBoard[this->curpoint + 7];     // Read the Instruction

    bigsize = (uint16_t) ((size_H << 8) | size_L); // Put together size, to get all data on buffer

    temp = ((this->CommsBoard[this->curpoint + 6 + bigsize] << 8)   // Combine read CRCs
           | this->CommsBoard[this->curpoint + 5 + bigsize]);       //

    if (update_crc(0, &this->CommsBoard[this->curpoint], bigsize + 5) != temp) {
        // Compare the read CRC, against the calculated CRC, if not equal then return error
        this->Flt = UART_Parity;

    } else {
        // If CRCs equal, then data is correct, indicate good data
        this->Flt = UART_NoFault;

        temp = bigsize - 3;                     // Recalculate the number of parameters in packet
        *size = temp;                           // Provide size to addressed "size"

        // Loop through parameter data in "CommsBoard" and put onto Receive buffer
        for (bigsize = 0; bigsize != temp; bigsize++)
            this->Receive->InputWrite(this->CommsBoard[this->curpoint + 8 + bigsize]);
    }

    this->CleanBoard();                 // Once complete, clear "CommsBoard" via "CleanBoard"
    this->State = Dynm_Idle;            // Return "State" back to "Idle"
    this->curpoint = 0;                 // Set "curpoint" back to start

    return(1);
}

_UARTDevFlt Dynamixel::PoleTransmitPkg(uint8_t ID, uint8_t Instr, uint8_t *param, uint16_t size) {
/**************************************************************************************************
 * Function will only run if the "State" of interface is set to "Speaking" otherwise will not run.
 * Will then cycle through the data stored on "CommsBoard" and push onto the UART interface.
 * <<NOTE>>
 * That this function is using the "Pole" version of the UART transmission, and will therefore
 * continually check if can transmit, and will only exit once ALL data has been transmitted.
 *
 * After transmission, will then clear "CommsBoard" so ready for next step.
 * "State" is set to "IDLE"
 *************************************************************************************************/
    if (this->RequstTransmission(ID, Instr, param, size) != UART_NoFault)
        return(this->Flt);

    while (this->State != Dynm_Speaking) {  this->PkgCode();  };
    // Loop function call of coding package, until the "State" of interface is set to "Speaking"

    // Cycle through data on "CommsBoard", and transmit via UART
    for (this->curpoint = 0; this->curpoint != this->maxpoint; this->curpoint++) {
        this->PoleSingleTransmit(this->CommsBoard[this->curpoint]);
    }

    this->CleanBoard();                 // Once complete, clear "CommsBoard" via "CleanBoard"
    this->curpoint = 0;                 // Set "curpoint" back to start

    this->State = Dynm_Idle;            // Set "State" to "Idle"

    return (this->Flt = UART_NoFault);  // If make it this far, then indicate no error with data
}

uint8_t Dynamixel::PoleReceievePkg(uint8_t *ID, uint8_t *Instr, uint16_t *size) {
/**************************************************************************************************
 * If the "State" of interface is set to "Speaking" then function will not run.
 *
 * If in either "Idle" or "Listening", then set "State" to "Listening" to lock "CommsBoard", and
 * initialise points to the start - Also clear the search state - "SrcState" to nothing found
 * "Nothing".
 *
 * Then start listening for data on UART. If any data is found, put onto "CommsBoard", update end
 * pointer "maxpoint" and then Search function.
 * <<NOTE>>
 * That this function is using the "Pole" version of the UART reading, and will therefore
 * continually check if anything has been read, and will only exit once a complete package has
 * been read.
 *
 * Only once a complete package has been received - "SrcState" is equal to "Complete" exit loop,
 * run Decode function, and then return state to "Idle".
 *************************************************************************************************/
    if (this->State == Dynm_Speaking)   // Check to see if state is equal to speaking
        return(0);                      // If it is then exit function

    this->State = Dynm_Listening;       // Otherwise force state to "Listening"

    // Put in the GPIO switch if this is used to manage the single wire UART

    this->maxpoint = 0;                 // Initialise "CommsBoard" pointers to 0
    this->curpoint = 0;                 // Initialise "CommsBoard" pointers to 0
    this->SrcState = Dynm_Nothing;      // Initialise search state to nothing found - "Nothing"

    while (this->SrcState != Dynm_Complete) {                   // Loop until complete packages has
                                                                // been found
        if (this->maxpoint < this->Length) {                        // Only add whilst within size
            this->CommsBoard[this->maxpoint++] = PoleSingleRead();  // of array

        } else { this->Flt = UART_DataError; }              // If outside size, set fault

        this->UpdateWaterMark();                // Update the watermark
        PkgSearch();        // Then search for valid package
    }

    return(this->PkgDecode(ID, Instr, size));                   // Decode package
}

_DynmSearch Dynamixel::PkgSearch(void) {
/**************************************************************************************************
 * Function will read through the current contents of "CommsBoard" and determine if a complete
 * package has been read - note that this will not do any CRC, as this function is to remain
 * "light", so can be used within an interrupt. The check of the CRC is done within "PkgDecode()"
 *
 * The function will only run if the "Search State" is set to "Nothing" i.e. no package has been
 * read, and there is atleast 3 pieces of data that have been read - as need atleast this to
 * determine, if a header has been sent.
 * Once these conditions have been met, then a loop will occur searching for the initial "HEADER"
 * as per the Dynamixel protocol. Once this has been found, the "curpoint" will be fixed at the
 * start of the read back packet - "Search State" will be set to "Header" indicating that this has
 * been found.
 * Once the "Header" has been found, function will wait for the next 6 pieces of data to be read,
 * such that the full size can be determined. Once this has been found, the function will then wait
 * for the size of the packet to align with the size provided by protocol.
 * Once has received this, will set the state of "Search" to "Complete"
 *************************************************************************************************/
    uint16_t size = 0;                  // Set variable to store 16bit size to zero

    // If no header has been found, and the amount of data read is greater than 3 continually
    // search for the "Header" of package to be found - first 3 bytes = FF, FF, FD
    if ((this->SrcState == Dynm_Nothing) && (this->maxpoint > 3)) {
        // Only when the search state is at "Nothing", and the amount of data in the buffer is
        // greater then 3, starting checking

        for (this->curpoint = 2; this->curpoint != this->maxpoint; this->curpoint++) {
            if ( (this->CommsBoard[this->curpoint - 2] == 0xFF) &&
                 (this->CommsBoard[this->curpoint - 1] == 0xFF) &&
                 (this->CommsBoard[this->curpoint]     == 0xFD) ) {
                this->SrcState = Dynm_Header;   // Have found Header now
                this->curpoint -= 2;            // Move pointer to start of package
                break;                          // Exit loop
            }
        }
    }

    // Once "Header" has been found, wait until full package has been read.
    if (this->SrcState == Dynm_Header) {
        // Within this function, the "curpoint" will be pointing to the start of the package
        if (this->maxpoint >= (this->curpoint + 6)) {   // If have read sufficient data to include
                                                        // the length then, calculate length
            size = (uint16_t) (this->CommsBoard[this->curpoint + 5]);
            size |= (uint16_t) ((this->CommsBoard[this->curpoint + 6]) << 8);
            if (this->maxpoint >= (this->curpoint + 7 + size)) {
                this->maxpoint = 7 + size;              // Re-calculate size of packages
                this->SrcState = Dynm_Complete;         // Set state as complete
            }
        }
    }

    // Return current state of search
    return(this->SrcState);
}

uint8_t Dynamixel::IRQPreHandle(uint8_t *ID, uint8_t *Instr, uint16_t *size) {
/**************************************************************************************************
 * INTERRUPTS:
 * This is a periodic function that will need to be called either frequently, such as to allow for
 * satisfactory communication with any Dynamixel device via Interrupts.
 *
 * This function will:
 *      - Populate the "CommsBoard" if it is free, with data to be transmitted
 *          > Enable the Transmission interrupt if in "Speaking" state
 *      - Decode and check any complete package read and stored onto "CommsBoard"
 *************************************************************************************************/
    this->PkgCode();                    // Call Package Code function
                                        // Function will only run if "State" = "Idle" and data is
                                        // requested to be transmitted

    if (this->State == Dynm_Speaking) { // If the "State" is set to "Speaking"
        this->TransmtIT(UART_Enable);   // Enable Transmit interrupt

            // Put in the GPIO switch if this is used to manage the single wire UART

    }

    return(this->PkgDecode(ID, Instr, size));       // If data has been read, and a complete
                                                    // package has been read, then decode
}

void Dynamixel::IRQHandle(void) {
/**************************************************************************************************
 * INTERRUPTS:
 * Interrupt Service Routine for the Dynamixel class. Each of the supported devices needs to call
 * this function in different ways - therefore each implementation is mentioned within the coded
 * section.
 *
 * Function will then read the hardware status flags, and determine which interrupts has been
 * triggered:
 *      If Transmission buffer empty has been enabled, then take next byte of data from
 *      "CommsBoard", and put onto hardware for transmission.
 *      If there is no other data to transmit, disable this interrupt - and ENABLE complete
 *      interrupt
 *
 *      If Transmission Complete has been enabled, then disable interrupt, and:
 *          Set "State" to "Listening" - to force reading of response
 *          Set "Search" to "Nothing"
 *          Initialise "CommsBoard" pointers to "0" - "Quick Clear"
 *          << Also required to change state of GPIO if used to manage single wire UART >>
 *
 *
 * Function will then read the hardware status flags, and determine which interrupt has been
 * triggered:
 *      If receive interrupt enabled, then data from Data Register is stored onto the "Receive"
 *      buffer
 *
 *      If transmission interrupt enabled, then data from "Transmit" buffer is put onto the Data
 *      Register for transmission.
 *
 *      No other interrupts are currently supported.
 *************************************************************************************************/

#if   (zz__MiEmbedType__zz == 50)       // If the target device is an STM32Fxx from cubeMX then
//=================================================================================================
/**************************************************************************************************
 * Example of call.
 *  As the main.h/main.c are included within the interrupt header and source file, the function
 *  call needs to be setup there.
 *  A global pointer to the Dynamixel class needs to be setup, then within the main() routine, it
 *  needs to be configured to the desired settings (STM32CubeMX - handle linked to class).
 *
 *  Then external function needs to call ".IRQHandle", and then be called within the interrupt
 *  function call:
 *
 *  main.c
 *      * Global pointer
 *      Dynamixel *DynxDevice;
 *
 *      main() {
 *      DynxDevice = new Dynamixel(&huart1);
 *      while() {};
 *      }
 *
 *      void UARTDevice_IRQHandler(void) {  DynxDevice->IRQHandle();  }
 *
 *  main.h
 *      extern "C" {
 *      void UARTDevice_IRQHandler(void);
 *      }
 *************************************************************************************************/
    uint8_t tempdata = 0x00;    // Temporary variable to store data for UART

    if (this->TransmitComptITCheck() == 0x01) { // Will be triggered when transmission is complete
        if (this->State == Dynm_Speaking) {
            this->State     = Dynm_Listening;

            // Put in the GPIO swith if this is used to manage the single wire UART

            this->SrcState  = Dynm_Nothing;

            this->maxpoint = 0;
            this->curpoint = 0;

            this->TransCmIT(UART_Disable);
        }
    }
    // If the Data Empty Interrupt has been triggered AND is enabled as Interrupt then...
    if(this->TransmitEmptyITCheck() == 0x01) {
        if (this->State == Dynm_Speaking) {
            // If there is data to be transmitted, then take from buffer and transmit
            if (this->curpoint != this->maxpoint) {
                this->DRWrite((uint8_t)this->CommsBoard[this->curpoint++]);
            } else {
                this->TransmtIT(UART_Disable);
                this->TransCmIT(UART_Enable);
            }
        }
    }

    // If the Receive Data Interrupt has been triggered AND is enabled as Interrupt then ...
    if(this->ReceiveDataToReadChk() == 0x01) {
        tempdata  = (uint8_t) this->DRRead();       // Read data and put onto temp variable

        if ( (this->State == Dynm_Idle) || (this->State == Dynm_Listening) ) {
            this->State = Dynm_Listening;               // Force state into "Listening"
            // Only put data onto the "CommsBoard" only when Search State - "SrcState" is not
            // complete
            if ( (this->SrcState != Dynm_Complete) ) {
                if (this->maxpoint < this->Length) {                // Only add whilst within size
                    this->CommsBoard[this->maxpoint++] = tempdata;  // of array

                } else { this->Flt = UART_DataError; }              // If outside size, set fault

                this->UpdateWaterMark();                // Update the watermark

                this->PkgSearch();     // Then search for valid package
            }
        }
    }

#elif (zz__MiEmbedType__zz == 10)       // If the target device is an Raspberry Pi then
//=================================================================================================
/**************************************************************************************************
 * Example of call.
 *  As setting up a real interrupt for Raspberry Pi involves hacking the kernel, which I am not
 *  doing, the route I have taken is to use threads - pthreads.
 *  What this involves is creating a separate stream (thread) which will just check the size of the
 *  UART peripheral buffer or if data has been requested to be sent (via pseudo interrupt
 *  register). Then the main thread can use the Read/Transmit functions as normal.
 *
 *  So setting up the pthread:
 *  The -pthread needs to be added to the G++ linker library, won't require the "-" within eclipse,
 *  however for "CMakeList", will need to be written as "-lpthread".
 *  Then within the main code add "include <pthread.h>
 *
 *  This will then allow you to create a new stream:
 *  main.c {
 *  pthread_t thread;   // Create thread handle
 *  if (pthread_create(&thread, NULL, &threadfunction, NULL) != 0) {
 *      std::cout << "Failed to create thread" << std::endl;
 *  }
 *
 * pthread_create will return 0 if it has created a new thread. In the example above "&thread" is
 * the thread handle created above, next entry ...no idea... 3rd entry is a pointer to the desired
 * function call, 4th can be used to provide values to the function - however I haven't tried this.
 *
 * void *threadfunction(void *value) {
 *  uint8_t UARTReturn;
 *  while (1) {
 *      delay(100);
 *      DynxDevice->IRQHandle();
 *  }
 *
 *  return 0;
 * }
 *
 * Similar to the STM32 a pointer to the Dynamixel will need to be made global to allow this
 * new thread to all the "IRQHandler"
 *************************************************************************************************/
    uint8_t tempdata = 0x00;    // Temporary variable to store data for UART

    if (this->TransmitComptITCheck() == 0x01) { // Will be triggered when transmission is complete
        if (this->State == Dynm_Speaking) {
            this->State     = Dynm_Listening;

            // Put in the GPIO swith if this is used to manage the single wire UART

            this->SrcState  = Dynm_Nothing;

            this->maxpoint = 0;
            this->curpoint = 0;

            this->TransCmIT(UART_Disable);
        }
    }
    // If the Data Empty Interrupt has been triggered AND is enabled as Interrupt then...
    if(this->TransmitEmptyITCheck() == 0x01) {
        if (this->State == Dynm_Speaking) {
            // If there is data to be transmitted, then take from buffer and transmit
            if (this->curpoint != this->maxpoint) {
                this->DRWrite((uint8_t)this->CommsBoard[this->curpoint++]);
            } else {
                this->TransmtIT(UART_Disable);
                this->TransCmIT(UART_Enable);
            }
        }
    }

    int BufferContents = 0;             // Variable to store the amount of data in UART peripheral

    // If the Receive Data Interrupt has been triggered AND is enabled as Interrupt then ...
    if(this->ReceiveDataToReadChk() == 0x01) {
        BufferContents = this->AnySerDataAvil();    // Get the amount of data in UART

        while (BufferContents > 0) {
            tempdata = (uint8_t) this->DRRead();    // Read data from array, this is done before
                                                    // the check of "state" to empty buffer if
                                                    // not in listening mode
            if ( (this->State == Dynm_Idle) || (this->State == Dynm_Listening) ) {
                this->State = Dynm_Listening;               // Force state into "Listening"
                // Only put data onto the "CommsBoard" only when Search State - "SrcState" is not
                // complete
                if ( (this->SrcState != Dynm_Complete) ) {
                    if (this->maxpoint < this->Length) {                // Only add whilst within
                        this->CommsBoard[this->maxpoint++] = tempdata;  // size of array

                    } else { this->Flt = UART_DataError; }          // If outside size, set fault

                    this->UpdateWaterMark();                // Update the watermark

                    this->PkgSearch();     // Then search for valid package
                }
            }
            BufferContents--;
        }
    }

#else
//=================================================================================================

#endif
}

Dynamixel::~Dynamixel()
{

}

uint16_t Dynamixel::update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr,
                               uint16_t data_blk_size)
{

    uint16_t i, j;

    uint16_t crc_table[256] = {
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

    for (j = 0; j < data_blk_size; j++)
    {
        i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}
