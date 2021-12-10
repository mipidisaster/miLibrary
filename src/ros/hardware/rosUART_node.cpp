/**************************************************************************************************
 * @file        rosUART_node.cpp
 * @author      Thomas
 * @brief       ROS node for the UART class
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * This node is called "uart_node"
 *   Configuration parameters:
 *      ~/config        /hardware_address
 *                      [ Text field to capture where the hardware file address is located
 *
 *                      /baud_rate
 *                      [ Number field for the specified bit rate of the UART device.
 *                      [ - Note, if none is provided will default to '115200bpm'
 *   Publishers:
 *      connection_status
 *                      [ Provides a count of the bytes received and sent via this bus. Also
 *                      [ calculates the instant transmission rate.
 *                      [ Refreshes every 0.5s (uses a callback timer for this)
 *
 *   Subscribers:
 *      None
 *
 *   Services:
 *      read_uart       [ Request-> Number of bytes to be read from hardware
 *                      [ Result -> Array (size as per requested) of data from hardware
 *                      [           Also captures the fault state of device (uint8_t)
 *                      [ Uses BUSctrl.srv
 *
 *      write_uart      [ Request-> Array of data to be sent
 *                      [ Result -> Fault status of hardware during write (uint8_t)
 *                      [ Uses BUSctrl.srv
 *
 *************************************************************************************************/
// C System Header(s)
// ------------------
#include <stdint.h>
#include <vector>       // for std::vector
#include <string>       // for strings
#include <algorithm>    // for std::count

// C++ System Header(s)
// --------------------
// None

// Other Libraries
// ---------------
#include "ros/ros.h"

// Project Libraries
// -----------------
#include "milibrary/Transmission.h"
#include "milibrary/BUSctrl.h"

#include "FileIndex.h"
// ~~~~~~~~~~~~~~~~~~~
#include FilIndROSNode_TP               // Template for the miROSnode file
#include FilInd_USART__HD               // Header for UART

//=================================================================================================
/**************************************************************************************************
 * Define any private variables
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

/**************************************************************************************************
 * Define any private function prototypes
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
class rosUART: public miROSnode {
private:
    // Constant(s) used within the class.
    std::string         khardware_baud_rate         = "baud_rate";

    std::string         kUART_read_service          = "read_uart";
    std::string         kUART_write_service         = "write_uart";

    std::string         kUART_publish_transmission  = "connection_status";

private:
    UARTPeriph              *_hardware_handle_;
    int                     _baud_rate_;

    std::vector<uint64_t>   _write_byte_count_;
    std::vector<float>      _write_byte_rate_;

    std::vector<uint64_t>   _read_byte_count_;
    std::vector<float>      _read_byte_rate_;

/**************************************************************************************************
 * == ROS STUFF == >>>   ROBOT OPERATING SYSTEM (ROS) OBJECTS    <<<
 *   -----------
 *  The follow are objects used for interfacing with the Robot Operating System.
 *************************************************************************************************/
    // PARAMETERS
    ////////////////////////
    // _file_location_ from miROSnode

    // MESSAGES
    ////////////////////////
    milibrary::Transmission _connection_status_message_;

    // PUBLISHERS
    ////////////////////////
    ros::Publisher          _connection_status_publisher_;

    // SUBSCRIBERS
    ////////////////////////

    // TIMERS
    ////////////////////////
    ros::Timer              _publisher_timer_;

    // SERVICES
    ////////////////////////
    ros::ServiceServer      _read_server_;
    ros::ServiceServer      _write_server_;

    // ACTIONS
    ////////////////////////

public:
/*
 *  @brief:  Create the UART node class, basic construction
 *
 *  @param:  Pointer to the 'NodeHandle'
 *  @param:  Pointer to the 'NodeHandle', setup for private - ("~")
 *  @retval: rosUART class
 */
    rosUART(ros::NodeHandle* normal, ros::NodeHandle* private_params):
    miROSnode(normal, private_params)
    {
        _hardware_handle_   = NULL;     // Initialise the pointer to NULL
        _baud_rate_         = 0;
    }

    /*
     *  @brief:  Setups the UART for the node, as per the expected input/configuration parameters
     *           from within the rosparam space.
     *           If there are any issues with the supplied values; which cannot be managed
     *           internally. Will return an error (value of -1), to be checked at the 'main' level
     *           to close the node down.
     *
     *  @param:  void
     *  @retval: integer value - 0 = no issues, -1 = issues detected
     */
    int configNode(void)
    {
        //=========================================================================================
        ParamStatus file_location_parameter_good = rosParamCheck(
                                                   kconfig_sub_area + khardware_address_param,
                                                   _file_location_);

        if          (file_location_parameter_good == ParamStatus::kParameter_present_but_invalid) {
            ROS_ERROR("File address ROS Parameter %s was not set to the correct type. "
                      "Expecting list type to be provided. i.e. \"/dev/serial0\"",
                      (kconfig_sub_area + khardware_address_param).c_str());

            return -1;
        } else if   (file_location_parameter_good == ParamStatus::kParameter_not_present) {
            ROS_ERROR("File address ROS Parameter %s was not within roscore",
                     (kconfig_sub_area + khardware_address_param).c_str());

            return -1;
        } else {
            int temp = _file_location_.size();

            if (_file_location_.size() == 0) {
                ROS_ERROR("File address ROS Parameter %s was not set to the correct type. "
                          "Expecting list type to be provided. i.e. \"/dev/serial0\"",
                          (kconfig_sub_area + khardware_address_param).c_str());
                return -1;
            } else {
                ROS_INFO("The following file address have been provided for "
                         "hardware interface: %s", _file_location_.c_str());
            }
        }
        //=========================================================================================
        ParamStatus baud_rate_parameter_good     = rosParamCheck(
                                                   kconfig_sub_area + khardware_baud_rate,
                                                   _baud_rate_);

        if (baud_rate_parameter_good != ParamStatus::kParameter_present) {
            ROS_WARN("No Baudrate has been provided via ROS Parameter %s, "
                     "defaulting to 115200bps",
                    (kconfig_sub_area + khardware_baud_rate).c_str());
            _baud_rate_ = 115200;
        }

        //=========================================================================================
        ROS_INFO("Checking that there is no overlap of hardware filenames...");
        if(duplicationParamCheck() < 0) {       // If duplication(s) detected
            return -1;                          // exit
        }

        _hardware_handle_ = new UARTPeriph(_file_location_.c_str(), _baud_rate_,
                                           __null, 0, __null, 0);

        ROS_INFO("UART has been setup");
        //=========================================================================================
        _connection_status_publisher_ = _nh_.advertise<milibrary::Transmission>(
                                                kUART_publish_transmission,
                                                20);

        //=========================================================================================
        _publisher_timer_ = _nh_.createTimer(ros::Duration(0.5),
                                             &rosUART::callbackUARTpublish, this, false);

        //=========================================================================================
        _read_server_       = _nh_.advertiseService(kUART_read_service,
                                                    &rosUART::callbackUARTread,
                                                    this);

        _write_server_      = _nh_.advertiseService(kUART_write_service,
                                                    &rosUART::callbackUARTwrite,
                                                    this);

        //=========================================================================================

        ROS_INFO("UART Server constructed");
        return 0;   // If got to this point, no errors were detected
    }

    /*
     *  @brief:  Callback function to be used as part of the service of this node. Will receive
     *           in the BUSctrl message, and will return the specified number of bytes read from
     *           the UART bus.
     *           Note - if the required number is not present, will wait till there is enough to
     *           fully the request.
     *           << Special Feature >>
     *              If the number to be read is specified as "read_size = 0", service will
     *              determine how much data is in the hardware buffer, and return this.
     *
     *  @param:  milibrary BUSctrl request
     *  @param:  milibrary BUSctrl response
     *  @retval: Service needs to return a boolean type
     */
    bool callbackUARTread(milibrary::BUSctrl::Request &req, milibrary::BUSctrl::Response &res) {
        uint32_t data_size = req.read_size;
        if (data_size == 0) {
            data_size = _hardware_handle_->anySerDataAvil();
        }

        while(_hardware_handle_->anySerDataAvil() < data_size) {};

        std::vector<uint8_t> uart_read_back;
        for (uint32_t i = 0; i != data_size; i++) {
            uart_read_back.push_back( _hardware_handle_->poleSingleRead() );

            // Count number of bytes received/read
            if (_read_byte_count_.size() != 0) {
                _read_byte_count_[0]++;
                _read_byte_count_[1]++;
            }
        }

        res.read_data   = uart_read_back;
        res.fault       = (uint8_t) _hardware_handle_->flt;
        return true;
    }

    /*
     *  @brief:  Callback function to be used as part of the service of this node. Will receive
     *           in the BUSctrl message, and will transmit the requested number of bytes via UART
     *           Note - if the requested number of bytes is '0', nothing will be transmitted.
     *
     *  @param:  milibrary BUSctrl request
     *  @param:  milibrary BUSctrl response
     *  @retval: Service needs to return a boolean type
     */
    bool callbackUARTwrite(milibrary::BUSctrl::Request &req, milibrary::BUSctrl::Response &res) {
        for (uint32_t i = 0; i != req.write_data.size(); i++) {
            _hardware_handle_->poleSingleTransmit(req.write_data.at(i));

            // Count number of bytes sent/written
            if (_write_byte_count_.size() != 0) {
                _write_byte_count_[0]++;
                _write_byte_count_[1]++;
            }
        }

        res.fault       = (uint8_t) _hardware_handle_->flt;
        return true;
    }

    /*
     *  @brief:  Callback function to be used to publish the Transmission rate of the hardware.
     *           Rate of transmission is based upon the timer function which calls this
     *
     *  @param:  ROS Timer event
     *  @retval: None
     */
    void callbackUARTpublish(const ros::TimerEvent& event) {
        // If this is the first pass of generating this message, then initialise it and publish
        if (_connection_status_message_.received_bytes.size() == 0) {
            _read_byte_count_.insert( _read_byte_count_.begin(),  2, 0);
            _write_byte_count_.insert(_write_byte_count_.begin(), 2, 0);

            _read_byte_rate_.insert(  _read_byte_rate_.begin(),   2, 0.00f);
            _write_byte_rate_.insert( _write_byte_rate_.begin(),  2, 0.00f);

            _connection_status_message_.received_bytes  = _read_byte_count_;
            _connection_status_message_.sent_bytes      = _write_byte_count_;

            _connection_status_message_.received_rates  = _read_byte_rate_;
            _connection_status_message_.sent_rates      = _write_byte_rate_;


            _connection_status_message_.header.seq = 0;
            _connection_status_message_.header.stamp = ros::Time::now();

            _connection_status_publisher_.publish(_connection_status_message_);
        }
        else {
            _connection_status_message_.header.seq++;

            ros::Time curTime = ros::Time::now();
            ros::Duration time_diff = curTime - _connection_status_message_.header.stamp;
            _connection_status_message_.header.stamp = curTime;

            uint64_t rate_difference;
            // Provide protection against counter overflow, unlikely to need this
            for (int i = 0; i != _read_byte_count_.size(); i ++) {
            if (_connection_status_message_.received_bytes[i] >= _read_byte_count_[i]) {
                rate_difference = _read_byte_count_[i] -
                                  _connection_status_message_.received_bytes[i];
            } else {
                rate_difference = ((uint64_t) -1) - _connection_status_message_.received_bytes[i];
                rate_difference += _read_byte_count_[i] + 1;
            }
            _connection_status_message_.received_bytes[i] = _read_byte_count_[i];
            _connection_status_message_.received_rates[i] = ( (float) rate_difference ) /
                                                            ((float) time_diff.toSec());
            }

            // Provide protection against counter overflow, unlikely to need this
            for (int i = 0; i != _write_byte_count_.size(); i ++) {
            if (_connection_status_message_.sent_bytes[i] >= _write_byte_count_[i]) {
                rate_difference = _write_byte_count_[i] -
                                  _connection_status_message_.sent_bytes[i];
            } else {
                rate_difference = ((uint64_t) -1) - _connection_status_message_.sent_bytes[i];
                rate_difference += _write_byte_count_[i] + 1;
            }
            _connection_status_message_.sent_bytes[i] = _write_byte_count_[i];
            _connection_status_message_.sent_rates[i] = ( (float) rate_difference ) /
                                                          ((float) time_diff.toSec());
            }

            _connection_status_publisher_.publish(_connection_status_message_);
        }
    }

    ~rosUART() {
        ROS_INFO("Shutting down the node, and killing functions");
        delete _hardware_handle_;
    }

};

//=================================================================================================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uart_basic");
    ros::NodeHandle n;
    ros::NodeHandle private_params("~");

    rosUART  node_UART(&n, &private_params);
    if (node_UART.configNode() < 0) {
        ROS_ERROR("Error detected during UART construction, exiting node...");
        return -1;
    }

    ROS_INFO("UART node ready for use");
    ros::spin();

    // On node shutdown, don't think it reaches this part of main()
    // However, will call class destroyer
    return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
 * ===============================================================================================
 * Local functions
 * ===============================================================================================
 *************************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////
// None

