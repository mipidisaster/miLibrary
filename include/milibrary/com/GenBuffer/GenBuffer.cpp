/**************************************************************************************************
 * @file        GenBuffer.hpp
 * @author      Thomas
 * @brief       Header file for the Generic GenBuffer Class handle (template)
 **************************************************************************************************
 @ attention

 << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * Class will create a circular buffer (queue) to store data from sources to be read at a later
 * time.
 * Use of class
 *      Initial call can either be empty or be setup with the desired size of the buffer.
 *          If it is constructed without any parameters, then the default size of the buffer will
 *          be set to 32 entries deep
 *
 *      Then data can be added to the buffer via ".inputWrite".
 *      ***NOTE***
 *          If the input pointer of data catches up to the output pointer (therefore buffer is
 *          full), the write will force the output pointer to increment by 1, to ensure that the
 *          data within the buffer is limited to only be the defined length old.
 *
 *      Read data from buffer via ".outputRead(<pointer>)", the read will only update the pointed
 *      data if there is new data within the buffer (i.e. not empty). If empty then it will not
 *      update pointer, and return the state "GenBuffer_Empty"
 *
 *      Quicker functions "quickWrite" and "quickRead", allow for cycling through the entries
 *      within the buffer and returning values.
 *
 *      ".flush" can be used to clear the entire contents of the buffer, and return all pointers
 *      back to the start of the buffer.
 *
 *      ".state" can be used to determine what the current state of the buffer is, returning a
 *      enumerate type of "_GenBufState" defined below.
 *
 *      ".readBuffer(<position>)" Return specified position within buffer.
 *
 *      ".spaceRemaining" and ".SpaceTilArrayEnd", can be used to determine how much size is
 *      remaining within the buffer. "SpaceTilArrayEnd", shows how many entries are left before
 *      the source array reaches the bottom (it includes the entry currently pointed too).
 *      ".UnreadCount" can be used to determine how many entries within the buffer have not been
 *      read yet.
 *
 *      ".writeErase" and ".readErase" are functions which can be used to quickly erase a
 *      specified number of entries either from the read/write side of the buffer.
 *      NOTE however, that if the number of entries to erase brings the write pointer pass the
 *           buffer being FULL. Then function will bring the pointer such that the state of the
 *           buffer will be FULL.
 *           Additionally, if the same is done for the read pointer, then instead it will just put
 *           the buffer into the EMPTY state.
 *
 *  This class has been defined within a template format (which is why the include at the end for
 *  the source file has been added - template call needs to include declaration and definition)
 *  So to use this class, it needs to be called like this:
 *      GenBuffer<--type--> testme(<option>);
 *
 *      --type-- can be changed for any type of variable uint8_t, uint16_t, etc. think this also
 *      include structures.
 *************************************************************************************************/
#ifndef GENBUFFER_TEMPLATE_         // As this class contains a template format, need to include
#define GENBUFFER_TEMPLATE_         // the source file within the header, therefore protection is
                                    // required from multiple loops.

#include "FileIndex.h"              // Not really needed for this source file, however kept for
                                    // traceability

#include <stdint.h>                 // Include standard integer entries

#if ( defined(zz__MiSTM32Fx__zz) || defined(zz__MiSTM32Lx__zz)  )
// If the target device is either STM32Fxx or STM32Lxx from cubeMX then ...
//=================================================================================================
// None

#elif defined(zz__MiRaspbPi__zz)        // If the target device is an Raspberry Pi then
//=================================================================================================
// None

#else
//=================================================================================================
// None

#endif

// Defines specific within this class
typedef enum {
    kGenBuffer_Empty    = 0,    // Indicates that the buffer is empty of any new data
    kGenBuffer_New_Data = 1,    // Indicates that the buffer contains new data to be read
    kGenBuffer_Full     = 2     // Indicates that the buffer is full, and no new data can be added
} _GenBufState;

template <typename Typ>
class GenBuffer {
public:
    // Declarations which are generic, and will be used in ALL devices
        uint16_t        input_pointer;      // Pointer to where the current input point is
        uint16_t        output_pointer;     // Pointer to where the current output point is

        uint16_t        length;             // Size of the buffer

        Typ             *pa;                // Points to the array (Buffer)

    public:
        void create(Typ *arrayloc, uint16_t size);

        GenBuffer(void);
        GenBuffer(Typ *arrayloc, uint16_t size);
        // As have defined that the "GenBuffer" needs to be "Lite", then use of "new" and "delete"
        // is not required, therefore a fully defined array is to be provided, and used within
        // the class.

        virtual ~GenBuffer();               // Destructor of class

        void flush(void);                   // Clear the data within the buffer
        void qFlush(void);                  // Clear buffer, by setting pointers to 0

        _GenBufState state(void);           // Function to determine state of buffer:
                                            // Full/NewData/Empty

        void inputWrite(Typ newdata);       // Add data onto the buffer (if at size of buffer, will
                                            // override oldest points (maintaining defined size)
        _GenBufState outputRead(Typ *readdata); // Read next data entry in buffer (if data is
                                                // present)
        Typ readBuffer(uint16_t position);      // Read specific entry from buffer

        uint16_t spaceRemaining(void);          // Return number of entries in buffer before, FULL
        uint16_t spaceTilArrayEnd(void);        // Return number of entries left till end of array

        uint16_t unreadCount(void);             // Return the number of un-read entries within
                                                // Buffer

        void quickWrite(Typ *newdata, uint16_t size);   // Take input array, and populate "size"
                                                        // into Buffer
        uint16_t quickRead(Typ *backdata, uint16_t size);
        // Provide array to retain read back data from buffer. Input size, limits the number of
        // entries returned. Returned value is the number of entries actually populated (to cater
        // for the buffer being empty; therefore output will not equal size).

        void writeErase(uint16_t size);         // Erase "size" number of data points from the
                                                // current position for the write/input buffer
                                                // "input_pointer"
        void readErase(uint16_t size);          // Erase "size" number of data points from the
                                                // current position for the read/output buffer
                                                // "output_pointer"
        // For both of these functions, if the size specified, brings the buffer past "FULL". Then
        // the specified pointers will be setup such that:
        //  input_pointer will be set to bring the buffer to "FULL"
        //  output_pointer will be set to bring the buffer to "EMPTY"

// Device specific entries
// None
};

template <typename Typ>
void GenBuffer<Typ>::flush(void) {
/**************************************************************************************************
 * Function goes through the contents of the buffer, and writes everything to "0", and then
 * returns the input/output pointers back to the start of the buffer - ready for new data
 *************************************************************************************************/
    uint16_t i;

    for (i = 0; i != length; i++) { // Start looping through buffer
        pa[i] = 0;                  // Write the data back to "0"
    }

    qFlush();                       // Flush the data to default values
}

template <typename Typ>
void GenBuffer<Typ>::qFlush(void) {
/**************************************************************************************************
 * Quick clear of the buffers, by setting all pointers to 0.
 *************************************************************************************************/
    output_pointer  = 0;            // Initialise pointers back to the start of the buffer
    input_pointer   = 0;            // Initialise pointers back to the start of the buffer
}

template <typename Typ>
GenBuffer<Typ>::GenBuffer() {
/**************************************************************************************************
 * Basic constructor of the class. Will initialise all pointers to zero, and the array pointer to
 * null.
 *************************************************************************************************/
    qFlush();                       // Flush the data to default values
    length = 0;                     // Initialise length to zero
    pa = __null;                    // Ensure that pointer is set to NULL
}

template <typename Typ>
void GenBuffer<Typ>::create(Typ *arrayloc, uint16_t size) {
/**************************************************************************************************
 * "Lite" function for GenBuffer.
 * Where the fully defined array is provided as input to constructor. This will then be linked to
 * internal class pointer "pa"
 *
 * Once done it will call the "QFlush" function to  setup pointers to the start of the buffer.
 *  Leaves the contents of the data unaffected.
 *************************************************************************************************/
    length = size;                  // Setup size of the buffer as per input
    pa = arrayloc;                  // Have pointer now point to input "arrayloc"

    qFlush();                       // Flush the data to default values
}

template <typename Typ>
GenBuffer<Typ>::GenBuffer(Typ *arrayloc, uint16_t size) {
/**************************************************************************************************
 * "Lite" function for GenBuffer.
 * Where the fully defined array is provided as input to constructor. This will then be linked to
 * internal class pointer "pa"
 *
 * Once done it will call the "QFlush" function to  setup pointers to the start of the buffer.
 *  Leaves the contents of the data unaffected.
 *************************************************************************************************/
    length = size;                  // Setup size of the buffer as per input
    pa = arrayloc;                  // Have pointer now point to input "arrayloc"

    qFlush();                       // Flush the data to default values
}

template <typename Typ>
_GenBufState GenBuffer<Typ>::state(void) {
/**************************************************************************************************
 * Function to determine the state of the Buffer - Empty/NewData/Full
 * It does this in 3 steps:
 *      1st     If the input pointer is equal to the output pointer. Then the buffer is empty
 *              and data can be added to the Buffer
 *
 *      2nd     If the output pointer is 1 in front of the input pointer, then the buffer is full
 *              (This does mean that the amount of data that can actually be written into the
 *              buffer is 1 minus the defined size)
 *
 *      3rd     If none of the above are true, then there is new data within the buffer which can
 *              be read
 *
 * Diagram:
 *                                     V Output pointer
 *             -------------------------------------------------------------------------------
 * Buffer ->  |    0    |    1    |    2    |    3    |    4    |    5    |    6    |    7    |
 *             -------------------------------------------------------------------------------
 *                           ^3        ^1        ^2  <- Input pointer
 *  Size is 8 deep
 *
 *      When the input pointer is at position 1 (equal to the Output pointer), then the buffer
 *      is EMPTY, and new data can be added
 *
 *      When the input pointer is at position 2 (just infront of Output pointer + anywhere else)
 *      then there is NEWDATA which needs to be read by the Output
 *
 *      When the input pointer is at position 3 (just behind the Output pointer), then the buffer
 *      is FULL. As data within entry [2] has NOT been read yet.
 *************************************************************************************************/
    if      (output_pointer == input_pointer)                   // If the pointers are equal
        return (kGenBuffer_Empty);                              // then buffer is empty

    else if (output_pointer == ((input_pointer + 1) % length))  // If output pointer is one behind
        return (kGenBuffer_Full);                               // the input, then buffer is full

    else                                                        // If none of the above are true
        return (kGenBuffer_New_Data);                           // then there is data in the buffer
                                                                // which needs to be read
}

template <typename Typ>
void GenBuffer<Typ>::inputWrite(Typ newdata) {
/**************************************************************************************************
 * Function will add data onto the buffer.
 * Then increase the input pointer, and limit it to the defined size of the buffer.
 * After this has been increased, if the buffer is considered "Empty" (input and output pointers
 * are equal) then need to increment the output pointer as well.
 * So as to maintain the oldest point of data within the buffer is limited to the size of the
 * buffer.
 *************************************************************************************************/
    pa[input_pointer] = newdata;                    // Add the input data into the buffer

    if (state() == kGenBuffer_Full) {
        output_pointer = (uint16_t) ( (output_pointer + 1) % length );
            // Increase the output pointer by 1, limited to size "length"
    }
    input_pointer = (uint16_t) ( (input_pointer + 1) % length );
        // Increment the input pointer, then take the

    // Modulus of this. This will then cause the input_pointer to be circled round if it equal to
    // the length.

    // If after the input pointer has been increased it is equal to the output pointer - therefore
    // is considered "Empty".
    // Then to force buffer size to be maintained as per "length", need to increase the output
    // pointer (essentially ensuring that the oldest point in the buffer is limited to "length"
    // old)

    // This part results in situations where an empty buffer can be turned into a FULL buffer
    // within a single write. Above correction seems to work...
    //if (State() == GenBuffer_Empty)                     // Check the buffer state, if equal to
                                                        // EMPTY, then
       // output_pointer = (output_pointer + 1) % length; // Increase the output pointer by 1,
                                                        // limited to size "length"
}

template <typename Typ>
_GenBufState GenBuffer<Typ>::outputRead(Typ *readdata) {
/**************************************************************************************************
 * Function will take data from the buffer.
 * It will only provide an updated output if the buffer contains data (i.e. is not empty), if it
 * is empty then it will attempt nothing and return the empty indication
 *
 * If there is data, again it will be linked to the pointed data. Then it will increase the output
 * pointer, and limit it to the defined size of the buffer.
 *************************************************************************************************/
    _GenBufState return_entry = state();    // Generate variable to store the current state of
                                            // buffer, and update with latest state

    if (!(return_entry == kGenBuffer_Empty)) {  // If the buffer is not empty (therefore there is
                                                // data be read
        *readdata = pa[output_pointer];         // Update the output with the latest entry from
                                                // buffer
        output_pointer = (uint16_t) ( (output_pointer + 1) % length );
            // Increase the output pointer by 1, limited to size "length"
        return(return_entry);               // Return state of buffer prior to read
    }
    else                                    // If state is equal to "Empty"
        return (return_entry);              // Return state of buffer (which will be "Empty")
}

template <typename Typ>
Typ GenBuffer<Typ>::readBuffer(uint16_t position) {
/**************************************************************************************************
 * Simple function to just read a specific entry within the buffer, whilst limiting it to the size
 * of the buffer.
 *************************************************************************************************/
    position %= length;         // Limit and loop the position to ensure it is within the buffer

    return (pa[position]);      // Return buffer entry
}

template <typename Typ>
uint16_t GenBuffer<Typ>::spaceRemaining(void) {
/**************************************************************************************************
 * Calculates the number of entries the buffer can take, before Buffer is FULL.
 *  Whereas the "InputWrite" will ensure that even if the buffer is full, it populates the latest
 *  data into the queue. This function will return "0" if the buffer is FULL, if the buffer is
 *  empty, then the output of this will be length - 1 (to cater for the buffer requiring 1 blank
 *  entry between input and output.
 *************************************************************************************************/
    if (state() == kGenBuffer_Empty) {
        return (length - 1);
    }
    else { return((uint16_t)( output_pointer - input_pointer + length - 1 ) % length); }
        // Difference between the output and input pointers (note input is likely to be larger
        // than the output), take this difference and add to the length
}

template <typename Typ>
uint16_t GenBuffer<Typ>::spaceTilArrayEnd(void) {
/**************************************************************************************************
 * Calculates the number of entries remaining within the source array, till the bottom is reached.
 *  At which point the GenBuffer will loop.
 *************************************************************************************************/
    return (length - input_pointer );
}

template <typename Typ>
uint16_t GenBuffer<Typ>::unreadCount(void) {
/**************************************************************************************************
 * Calculates the number of entries within the buffer which have not been read yet.
 * If the buffer is already full, then it will return the full buffer size.
 *************************************************************************************************/
    if (state() == kGenBuffer_Full) {
        return (length - 1);
    }
    else { return((uint16_t)( length + input_pointer - output_pointer ) % length); }
}

template <typename Typ>
void GenBuffer<Typ>::quickWrite(Typ *newdata, uint16_t size) {
/**************************************************************************************************
 * Populates the entries from "newdata" and puts into the GenBuffer. Only "size" entries are
 * copied.
 *  Relies upon the function "InputWrite"
 *************************************************************************************************/
    while(size != 0) {              // Cycle through the number of requested inputs
        inputWrite(*newdata);       // Put into array (also does loop back)
        newdata++;                  // Increase data pointer
        size--;                     // Decrease size (till = 0)
    }
}

template <typename Typ>
uint16_t GenBuffer<Typ>::quickRead(Typ *backdata, uint16_t size) {
/**************************************************************************************************
 * Goes through the contents of the Buffer, and returns the data into the return array "backdata".
 * Will only cycle through "size" number of entries, actual entries returned is captured within
 * "retsize"
 *
 * Populates the entries from "backdata" and puts into the GenBuffer. Only "size" entries are
 * copied.
 *  Relies upon the function "InputWrite"
 *************************************************************************************************/
    uint16_t return_size = 0;   // Initialise a count of the entries returned.
    Typ Buf_Entry = 0;          // Data variable to retain buffer entry

    while(size != 0) {          // Cycle through the number of data points to return
        if (outputRead(&Buf_Entry) != kGenBuffer_Empty) {   // If buffer is not empty
            *backdata = Buf_Entry;                          // Return data to input array
            backdata++;                                 // Increase data pointer
            return_size++;                              // Increment the return size count
            size--;                                     // Decrease size (till = 0)
        }
        else                                        // If buffer is empty
            break;                                  // exit loop
    }                                               // return size count will already be updated

    return (return_size);                   // Return the number of entries populated
}

template <typename Typ>
void GenBuffer<Typ>::writeErase(uint16_t size) {
/**************************************************************************************************
 * Bring the input_pointer forward by "size" number of entries - erasing "size" number of buffer
 * entries from being written too.
 *************************************************************************************************/
    if (spaceRemaining() >= size) {
        input_pointer = (input_pointer + size) % length;
        // Bring the input pointer forward by specified amount. So long as the space remaining is
        // enough to allow this.
    }
    else
        input_pointer = (input_pointer + spaceRemaining()) % length;
    // Otherwise, bring the input_pointer such that it is at the FULL threshold of the buffer
}

template <typename Typ>
void GenBuffer<Typ>::readErase(uint16_t size) {
/**************************************************************************************************
 * Bring the output_pointer forward by "size" number of entries - erasing "size" number of buffer
 * entries from being read from.
 *************************************************************************************************/
    if (unreadCount() >= size) {
        output_pointer = (output_pointer + size) % length;
        // Bring the input pointer forward by specified amount. So long as the space remaining is
        // enough to allow this.
    }
    else
        output_pointer = input_pointer;
    // Otherwise, bring the input_pointer such that it is at the FULL threshold of the buffer
}

template <typename Typ>
GenBuffer<Typ>::~GenBuffer() {
/**************************************************************************************************
 * When the destructor is called, need to ensure that the memory allocation is cleaned up, so as
 * to avoid "memory leakage"
 *************************************************************************************************/

}

#endif
