/**************************************************************************************************
 * @file        rosSPI_node.cpp
 * @author      Thomas
 * @brief       ROS node for the SPI class
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * This node is called "spi_node"
 *   Configuration parameters:
 *      ~/config        /hardware_address
 *                      [ Text field to capture where the hardware file address is located
 *
 *                      /baud_rate
 *                      [ Number field for the specified bit rate of the SPI device.
 *                      [ - Note, if none is provided will default to '4000000bps'
 *
 *                      /mode
 *                      [ State the SPI mode to be used, expecting a value of 0 -> 3.
 *
 *   [#] Device selection options
 *                      ~/config/gpio/gpio_pin
 *                      [ Provide a number of GPIO pins for extra chip selections. Each is assumed
 *                      [ to be connected to an external device.
 *                      [ The hardwired Chip Select is assumed to also be connected to a seperate
 *                      [ device.
 *                      [ Device selection available will be number of GPIOs provided + 1
 *
 *                      ~/config/demux/mux/gpio_pin
 *                                     low_reset/gpio_pin
 *                                     high_reset/gpio_pin
 *                      [ Provide configuration for an external Demultiplexor. The low_/high_reset
 *                      [ pins are optional; however only one GPIO is expected for each.
 *                      [ If this option is detected the '/mux/' parameter is mandatory.
 *                      [ Device selection available will be 2^(GPIOs provided)
 *
 *
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
 *      transfer_spi    [ Request-> Number of bytes to be transfered to selected device (address)
 *                      [ Result -> Array (equal to size of bytes written) of data from hardware
 *                      [           Also captures the fault state of device (uint8_t)
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
#include FilInd_GPIO___HD               // Allow use of GPIO class, for Chip Select
#include FilInd_DeMux__HD               // Allow use of DeMux class, for Chip Select
#include FilInd_SPIPe__HD               // Include class for SPI Peripheral

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
class rosSPI: public miROSnode {
private:
    // Constant(s) used within the class.
    std::string         khardware_baud_rate         = "baud_rate";
    std::string         khardware_mode              = "mode";

    std::string         koption_gpios               = "gpio/";

    std::string         koption_demux_mux           = "demux/mux/";
    std::string         koption_demux_l_reset       = "demux/low_reset/";
    std::string         koption_demux_h_reset       = "demux/high_reset/";

    std::string         kSPI_transfer_service       = "transfer_spi";

    std::string         kSPI_publish_transmission   = "connection_status";

private:
    SPIPeriph               *_hardware_handle_;
    SPIPeriph::SPIMode      _mode_;
    int                     _baud_rate_;

    int                     _address_options_;

    std::vector<uint64_t>   _write_byte_count_;
    std::vector<float>      _write_byte_rate_;

    std::vector<uint64_t>   _read_byte_count_;
    std::vector<float>      _read_byte_rate_;

    enum class nodeMode : uint8_t {
        kHardware_Chip_select               = 0x00,
        kSoftware_GPIOs_Chip_select         = 0x01,
        kSoftware_Demux_Chip_select         = 0x02,

        kMode_Selection_Error               = 0xFF
    }                       _node_mode_;

    std::vector<GPIO>       _gpio_cs_;
    GPIO                    *_demux_l_reset_;
    GPIO                    *_demux_h_reset_;
    DeMux                   *_demux_handle_;

/**************************************************************************************************
 * == ROS STUFF == >>>   ROBOT OPERATING SYSTEM (ROS) OBJECTS    <<<
 *   -----------
 *  The follow are objects used for interfacing with the Robot Operating System.
 *************************************************************************************************/
    // PARAMETERS
    ////////////////////////
    // _file_location_ from miROSnode
    std::vector<int>        _options_gpio_cs_params_;
    std::vector<int>        _options_demux_cs_params_;
    std::vector<int>        _options_demux_l_resetparams_;
    std::vector<int>        _options_demux_h_resetparams_;

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
    ros::ServiceServer      _transfer_server_;

    // ACTIONS
    ////////////////////////

public:
/*
 *  @brief:  Create the SPI node class, basic construction
 *
 *  @param:  Pointer to the 'NodeHandle'
 *  @param:  Pointer to the 'NodeHandle', setup for private - ("~")
 *  @retval: rosSPI class
 */
    rosSPI(ros::NodeHandle* normal, ros::NodeHandle* private_params):
    miROSnode(normal, private_params)
    {
        _hardware_handle_   = NULL;     // Initialise the pointer to NULL
        _baud_rate_         = 0;
        _mode_              = SPIPeriph::SPIMode::kMode0;

        _address_options_   = 0;

        _demux_l_reset_     = NULL;
        _demux_h_reset_     = NULL;
        _demux_handle_      = NULL;

        _node_mode_         = nodeMode::kHardware_Chip_select;
    }

    /*
     *  @brief:  Looks into the expected parameters needed for this node, and will check to see if
     *           they are valid.
     *
     *  @param:  void
     *  @retval: Integer value - 0 = no issues, -1 = issues detected
     */
    int checkSPIInputParameters(void)
    {
        ParamStatus file_location_parameter_good = rosParamCheck(
                                                   kconfig_sub_area + khardware_address_param,
                                                   _file_location_);

        if          (file_location_parameter_good == ParamStatus::kParameter_present_but_invalid) {
            ROS_ERROR("File address ROS Parameter %s was not set to the correct type. "
                      "Expecting list type to be provided. i.e. \"/dev/spidev0.0\"",
                      (kconfig_sub_area + khardware_address_param).c_str());

        }
        else if   (file_location_parameter_good == ParamStatus::kParameter_not_present) {
            ROS_ERROR("File address ROS Parameter %s was not within roscore",
                     (kconfig_sub_area + khardware_address_param).c_str());

        }
        else {
            int temp = _file_location_.size();

            if (_file_location_.size() == 0) {
                ROS_ERROR("File address ROS Parameter %s was not set to the correct type. "
                          "Expecting list type to be provided. i.e. \"/dev/spidev0.0\"",
                          (kconfig_sub_area + khardware_address_param).c_str());

                file_location_parameter_good = ParamStatus::kParameter_present_but_invalid;

            }
            else {
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
                     "defaulting to 4000000bps",
                    (kconfig_sub_area + khardware_baud_rate).c_str());
            _baud_rate_ = 4000000;
        }

        //=========================================================================================
        int temp_mode;
        ParamStatus mode_parameter_good          = rosParamCheck(
                                                   kconfig_sub_area + khardware_mode,
                                                   temp_mode);

        if (mode_parameter_good != ParamStatus::kParameter_present) {
            ROS_ERROR("SPI Mode is not present within roscore, this needs to be provided via %s",
                      (kconfig_sub_area + khardware_mode).c_str());

        }
        else {
            if ( (temp_mode < 0) || (temp_mode > 3) ) {
                ROS_ERROR("SPI mode has been specified incorrectly, expecing a value of "
                          "'0'/'1'/'2'/'3' for the respective modes");

                mode_parameter_good = ParamStatus::kParameter_present_but_invalid;
            }
            else {
                ROS_INFO("SPI mode %i selected", temp_mode);

                _mode_ = (SPIPeriph::SPIMode) temp_mode;
            }
        }

        //=========================================================================================
        // Determine if the parameters provided are sufficient for node construction to complete
        //
        if ( (file_location_parameter_good != ParamStatus::kParameter_present) ||
             (mode_parameter_good          != ParamStatus::kParameter_present) ) {
            ROS_ERROR("Errors detected within input ROS parameters, exiting node generation");
            return -1;
        }

        return 0;
    }

    /*
     *  @brief:  Will check the GPIOs provided as Software Chip Select (either GPIO, or DeMux),
     *           and display a message of what has been provided.
     *
     *  @param:  ParamStatus of the ROS parameter read check
     *  @param:  ROS Parameter key, so as to provide correct message
     *  @param:  Integer Vector to retrieve the GPIO numbers for message
     *  @retval: Integer value - 0 = no issues, -1 = issues detected
     */
    int constructGPIOoption(ParamStatus parameter_status, const std::string& key,
                            std::vector<int>& gpio_param)
    {
        if          (parameter_status == ParamStatus::kParameter_present_but_invalid) {
            ROS_ERROR("GPIO Chip Select ROS Parameter %s was not set to the correct type. "
                      "Expecting list type to be provided. i.e. \"[21, 20, 16, 26]\"",
                      (kconfig_sub_area + key + kgpio_generic_param).c_str());
            return -1;
        }
        //=========================================================================================
        // This option will never be used - but kept to keep consistancy between functions
        //
        else if   (parameter_status == ParamStatus::kParameter_not_present) {
            ROS_WARN("Output ROS Parameter %s was not within roscore",
                    (kconfig_sub_area + key + kgpio_generic_param).c_str());
            return 0;
        }
        //=========================================================================================
        else {
            ROS_INFO("The following gpios have been provided as output pins:");

            for (int i = 0; i != gpio_param.size(); i++) {
                ROS_INFO("    GPIO[%i]", gpio_param.at(i));
            }
        }

        return 0;
    }

    /*
     *  @brief:  Looks into the expected options provided for supporting multiple external devices
     *           attached to the same bus.
     *           Needs to determine which options (if any) have been selected, and disregard
     *           return fault(s) for any incomplete/incorrect inputs.
     *
     *  @param:  void
     *  @retval: Integer value - 0 = no issues, -1 = issues detected
     */
    int checkSPIAddressOptions(void)
    {
        ParamStatus option_gpio         = rosParamCheck(
                                    kconfig_sub_area + koption_gpios + kgpio_generic_param,
                                    _options_gpio_cs_params_);

        _output_gpio_params_.insert(_output_gpio_params_.begin(),
                _options_gpio_cs_params_.begin(), _options_gpio_cs_params_.end());

        // If there is a valid or partially valid GPIO pin setup. Then put node into Software
        // Driven GPIO Chip Selection mode
        //=========================================================================================
        if (option_gpio != ParamStatus::kParameter_not_present)
            _node_mode_ = nodeMode::kSoftware_GPIOs_Chip_select;

        ParamStatus option_demux_cs         = rosParamCheck(
                                    kconfig_sub_area + koption_demux_mux + kgpio_generic_param,
                                    _options_demux_cs_params_);

        _output_gpio_params_.insert(_output_gpio_params_.begin(),
                _options_demux_cs_params_.begin(), _options_demux_cs_params_.end());

        ParamStatus option_demux_l_cs       = rosParamCheck(
                                    kconfig_sub_area + koption_demux_l_reset + kgpio_generic_param,
                                    _options_demux_l_resetparams_);

        _output_gpio_params_.insert(_output_gpio_params_.begin(),
                _options_demux_l_resetparams_.begin(), _options_demux_l_resetparams_.end());

        ParamStatus option_demux_h_cs       = rosParamCheck(
                                   kconfig_sub_area + koption_demux_h_reset + kgpio_generic_param,
                                   _options_demux_h_resetparams_);

        _output_gpio_params_.insert(_output_gpio_params_.begin(),
                _options_demux_h_resetparams_.begin(), _options_demux_h_resetparams_.end());

        // If there is a valid or partially valid Demux setup. Then put node into Software
        // Driven Demux Chip Selection mode
        //      However, check to see if not in GPIO mode first; otherwise an invalid setup is
        //      provided.
        //=========================================================================================
        if ( (option_demux_cs   != ParamStatus::kParameter_not_present) ||
             (option_demux_l_cs != ParamStatus::kParameter_not_present) ||
             (option_demux_h_cs != ParamStatus::kParameter_not_present) ) {
            if (_node_mode_ == nodeMode::kSoftware_GPIOs_Chip_select) {
                _node_mode_ = nodeMode::kMode_Selection_Error;
            }
            else {
                _node_mode_ = nodeMode::kSoftware_Demux_Chip_select;
            }
        }

        int return_value = 0;

        if      (_node_mode_ == nodeMode::kMode_Selection_Error ) {
            ROS_ERROR("Unable to determine what SPI selection options you have selected; "
                      "something has been provided for both GPIOs and Demux Chip Selection. "
                      "Exiting node construction");
            _address_options_ = 0;
            return -1;
        }
        ///////////////////////////////////////////////////////////////////////////////////////////
        // Above if condition will detect if there is any overlap of options being select
        // Following statements will now check to see if the input parameters are acceptable for
        // use.
        else if (_node_mode_ == nodeMode::kSoftware_GPIOs_Chip_select ) {
            ROS_INFO("Software controlled SPI Chip Select option has been detected");
            return_value += constructGPIOoption(option_gpio,
                                                koption_gpios,
                                                _options_gpio_cs_params_);

            _address_options_ = _options_gpio_cs_params_.size() + 1;
        }

        else if (_node_mode_ == nodeMode::kSoftware_Demux_Chip_select ) {
            ROS_INFO("Software controlled SPI Chip Select - via DeMux - option has been detected");
            if (option_demux_cs == ParamStatus::kParameter_not_present) {
                ROS_ERROR("DeMux selection pins have not been provided, these are mandatory for "
                          "the option selected");
                return_value = -1;
            }
            else {
                return_value += constructGPIOoption(option_demux_cs,
                                                    koption_demux_mux,
                                                    _options_demux_cs_params_);

                _address_options_ = (1 << _options_demux_cs_params_.size());
            }

            return_value += constructGPIOoption(option_demux_l_cs,
                                                koption_demux_l_reset,
                                                _options_demux_l_resetparams_);
            return_value += constructGPIOoption(option_demux_h_cs,
                                                koption_demux_h_reset,
                                                _options_demux_h_resetparams_);


            if ( (_options_demux_l_resetparams_.size() > 1) ||
                 (_options_demux_h_resetparams_.size() > 1) ) {
                ROS_ERROR("More than one GPIO has been provided for one or both of the reset "
                          "connections in the DeMux. Only one GPIO pin is supported for each");
                return_value = -1;
            }
        }

        else {
            ROS_WARN("No additional options have been provided for SPI Chip Selection, "
                     "therefore have defaulted to HARDWARE DRIVEN chip selection");
            _address_options_ = 1;
        }

        if (return_value == 0) {
            ROS_INFO("Number of available external devices to communicate with %i",
                     _address_options_);
            return 0;
        }

        return -1;
    }

    /*
     *  @brief:  Setups the SPI for the node, as per the expected input/configuration parameters
     *           from within the rosparam space.
     *           If there are any issues with the supplied values; which cannot be managed
     *           internally. Will return an error (value of -1), to be checked at the 'main' level
     *           to close the node down.
     *
     *  @param:  void
     *  @retval: Integer value - 0 = no issues, -1 = issues detected
     */
    int configNode(void)
    {
        //=========================================================================================
        if (checkSPIInputParameters() < 0) {
            return -1;
        }

        if (checkSPIAddressOptions() < 0) {
            return -1;
        }

        //=========================================================================================
        ROS_INFO("Checking that there is no overlap of hardware filenames...");
        if(duplicationParamCheck() < 0) {       // If duplication(s) detected
            return -1;                          // exit
        }

        _hardware_handle_ = new SPIPeriph(_file_location_.c_str(), _baud_rate_, _mode_,
                                          __null, 0);

        if      (_node_mode_ == nodeMode::kSoftware_GPIOs_Chip_select) {
            GPIO::piSetup();        // Needed to make GPIO pins work
            for (int i = 0; i != _options_gpio_cs_params_.size(); i++) {
                _gpio_cs_.push_back(   GPIO(   GPIO::State::kHigh,
                                               _options_gpio_cs_params_.at(i),
                                               GPIO::Dir::kOutput)  );
            }
        }
        else if (_node_mode_ == nodeMode::kSoftware_Demux_Chip_select) {
            GPIO::piSetup();        // Needed to make GPIO pins work
            for (int i = 0; i != _options_demux_cs_params_.size(); i++) {
                _gpio_cs_.push_back(   GPIO(   GPIO::State::kLow,
                                               _options_demux_cs_params_.at(i),
                                               GPIO::Dir::kOutput)  );
            }

            if (_options_demux_h_resetparams_.size() == 1) {
                _demux_h_reset_ = new GPIO(GPIO::State::kLow,
                                           _options_demux_h_resetparams_.at(0),
                                           GPIO::Dir::kOutput) ;
            }
            else {
                _demux_h_reset_ = NULL;
            }

            if (_options_demux_l_resetparams_.size() == 1) {
                _demux_l_reset_ = new GPIO(GPIO::State::kHigh,
                                           _options_demux_l_resetparams_.at(0),
                                           GPIO::Dir::kOutput) ;
            }
            else {
                _demux_l_reset_ = NULL;
            }

            _demux_handle_ = new DeMux(_demux_h_reset_, _demux_l_reset_,
                                       _gpio_cs_.data(),
                                       _gpio_cs_.size());
        }

        ROS_INFO("SPI has been setup");
        //=========================================================================================
        _connection_status_publisher_ = _nh_.advertise<milibrary::Transmission>(
                                                kSPI_publish_transmission,
                                                20);

        //=========================================================================================
        _publisher_timer_ = _nh_.createTimer(ros::Duration(0.5),
                                             &rosSPI::callbackSPIpublish, this, false);

        //=========================================================================================
        _transfer_server_   = _nh_.advertiseService(kSPI_transfer_service,
                                                    &rosSPI::callbackSPItransfer,
                                                    this);

        //=========================================================================================

        ROS_INFO("SPI Server constructed");
        return 0;   // If got to this point, no errors were detected
    }

    /*
     *  @brief:  Callback function to be used as part of the service of this node. Will receive
     *           in the BUSctrl message, and facilitate communication to specified SPI Slave
     *           device.
     *           Note - Ignores parameter 'read_size' as data needs to be provided to send to
     *           device.
     *           Will also do nothing if 'write_data' is empty.
     *
     *  @param:  milibrary BUSctrl request
     *  @param:  milibrary BUSctrl response
     *  @retval: Service needs to return a boolean type
     */
    bool callbackSPItransfer(milibrary::BUSctrl::Request &req, milibrary::BUSctrl::Response &res) {
        if ( (req.address < 0) || (req.address >= _address_options_ ) ) {
            // Do something...
            return true;    // exit without doing anything
        }

        // Create an array of size equal to input 'write_data'
        std::vector<uint8_t> spi_read_back (req.write_data.size(), 0);

        if      ( (_node_mode_ == nodeMode::kSoftware_GPIOs_Chip_select) &&
                   (req.address != 0) ) {
            _hardware_handle_->poleMasterTransfer(&_gpio_cs_[req.address - 1],
                                                  req.write_data.data(), spi_read_back.data(),
                                                  req.write_data.size());
        }
        else if (_node_mode_ == nodeMode::kSoftware_Demux_Chip_select) {
            _hardware_handle_->poleMasterTransfer(_demux_handle_, req.address,
                                                  req.write_data.data(), spi_read_back.data(),
                                                  req.write_data.size());
        }
        else {
            _hardware_handle_->poleMasterTransfer(
                                                  req.write_data.data(), spi_read_back.data(),
                                                  req.write_data.size());
            req.address  =  0;
        }

        // Count number of bytes received/read
        if (_read_byte_count_.size() != 0) {
            _read_byte_count_[0]                += req.write_data.size();
            _read_byte_count_[req.address + 1]  += req.write_data.size();
        }

        // Count number of bytes sent/written
        if (_write_byte_count_.size() != 0) {
            _write_byte_count_[0]               += req.write_data.size();
            _write_byte_count_[req.address + 1] += req.write_data.size();
        }

        res.read_data   = spi_read_back;
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
    void callbackSPIpublish(const ros::TimerEvent& event) {
        // If this is the first pass of generating this message, then initialise it and publish
        if (_connection_status_message_.received_bytes.size() == 0) {
            _read_byte_count_.insert( _read_byte_count_.begin(),  (_address_options_ + 1), 0);
            _write_byte_count_.insert(_write_byte_count_.begin(), (_address_options_ + 1), 0);

            _read_byte_rate_.insert(  _read_byte_rate_.begin(),   (_address_options_ + 1), 0.00f);
            _write_byte_rate_.insert( _write_byte_rate_.begin(),  (_address_options_ + 1), 0.00f);

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
            }
            else {
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
            }
            else {
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

    ~rosSPI() {
        ROS_INFO("Shutting down the node, and killing functions");
        delete _hardware_handle_;
        delete _demux_handle_;
        delete _demux_l_reset_;
        delete _demux_h_reset_;
    }

};

//=================================================================================================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "spi_basic");
    ros::NodeHandle n;
    ros::NodeHandle private_params("~");

    rosSPI  node_SPI(&n, &private_params);
    if (node_SPI.configNode() < 0) {
        ROS_ERROR("Error detected during SPI construction, exiting node...");
        return -1;
    }

    ROS_INFO("SPI node ready for use");
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

