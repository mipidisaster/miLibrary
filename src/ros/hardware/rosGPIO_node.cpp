/**************************************************************************************************
 * @file        rosGPIO_node.cpp
 * @author      Thomas
 * @brief       ROS node for the GPIO class
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * This node is called "gpio_node"
 *   Configuration parameters:
 *      ~/config        /output/gpio_pin
 *                      /input/gpio_pin
 *                      [ Both of these parameters are expected to be lists of gpios pin numbers,
 *                      [ left most entry is the LSB
 *
 *                      /loop_rate
 *                      [ Floating data value, specifying the rate that the GPIO input state is
 *                      [ interrogated.
 *                      [ - Note, if none is provided will default to '10Hz'
 *                      [ - Note, Will only be published if there is a GPIO input parameter
 *
 *   Publishers:
 *      get_gpio
 *                      [ Will return the state of the GPIO input array
 *                      [ Uses std_msgs/UInt64.msg
 *                      [ - Note, Will only be published if there is a GPIO input parameter
 *
 *   Subscribers:
 *      None
 *
 *   Services:
 *      set_gpio
 *                      [ Request-> Value for the GPIO output to be set too
 *                      [ Result -> Nothing
 *                      [ Uses milibrary/srv/GPIOctrl.srv
 *
 *************************************************************************************************/
// C System Header(s)
// ------------------
#include <stdint.h>
#include <vector>       // for std::vector
#include <string>       // for strings

// C++ System Header(s)
// --------------------
// None

// Other Libraries
// ---------------
#include <ros/ros.h>
#include <ros/callback_queue.h>

// Project Libraries
// -----------------
#include "milibrary/GPIOctrl.h"
#include "std_msgs/UInt64.h"

#include "FileIndex.h"
// ~~~~~~~~~~~~~~~~~~~
#include FilIndROSNode_TP               // Template for the miROSnode file
#include FilInd_GPIO___HD               // Include the GPIO class handler

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
class rosGPIO: public miROSnode {
private:
    // Constant(s) used within the class.
    std::string         kinput_gpio_param           = "input/";
    std::string         koutput_gpio_param          = "output/";

    std::string         knode_loop_rate             = "loop_rate";

    std::string         kGPIO_set_service           = "set_gpio";

    std::string         kGPIO_publish_input_state   = "get_gpio";

private:
    std::vector<GPIO>   _output_array_;
    std::vector<GPIO>   _input_array_;

/**************************************************************************************************
 * == ROS STUFF == >>>   ROBOT OPERATING SYSTEM (ROS) OBJECTS    <<<
 *   -----------
 *  The follow are objects used for interfacing with the Robot Operating System.
 *************************************************************************************************/
    ros::NodeHandle         _nh_hardware_;
    ros::CallbackQueue      _hardware_callback_queue_;

    // PARAMETERS
    ////////////////////////
    // _input_gpio_params_ and _output_gpio_params_ from miROSnode
    double                          _loop_rate_parameter_;

    // MESSAGES
    ////////////////////////

    // PUBLISHERS
    ////////////////////////
    ros::Publisher          _gpio_input_publisher_;

    // SUBSCRIBERS
    ////////////////////////

    // TIMERS
    ////////////////////////
    ros::Timer              _gpio_output_check_timer_;

    // SERVICES
    ////////////////////////
    ros::ServiceServer      _set_server_;

    // ACTIONS
    ////////////////////////

public:
/*
 *  @brief:  Create the GPIO node class, basic construction
 *
 *  @param:  Pointer to the 'NodeHandle'
 *  @param:  Pointer to the 'NodeHandle', setup for private - ("~")
 *  @retval: rosGPIO class
 */
    rosGPIO(ros::NodeHandle* normal, ros::NodeHandle* private_namespace):
    miROSnode(normal, private_namespace)
    {
        _nh_hardware_.setCallbackQueue(&_hardware_callback_queue_);

        if (configNode() < 0) {
            ROS_ERROR("Error detected during GPIO construction, exiting node...");
            return;
        }

        nodeLoop();
    }

    /*
     *  @brief:  Looks into the expected parameters needed for this node, and will check to see if
     *           they are valid.
     *
     *  @param:  void
     *  @retval: Integer value - 0 = no issues, -1 = issues detected
     */
    int checkGPIOInputParameters(void)
    {
        ParamStatus output_parameters_good = rosParamCheck(
                                kconfig_sub_area + koutput_gpio_param + kgpio_generic_param,
                                _output_gpio_params_);

        if          (output_parameters_good == ParamStatus::kParameter_present_but_invalid) {
            ROS_ERROR("Output ROS Parameter %s was not set to the correct type. Expecting list "
                      "type to be provided. i.e. \"[21, 20, 16, 26]\"",
                      (kconfig_sub_area + koutput_gpio_param + kgpio_generic_param).c_str());
        }
        else if   (output_parameters_good == ParamStatus::kParameter_not_present) {
            ROS_WARN("Output ROS Parameter %s was not within roscore",
                    (kconfig_sub_area + koutput_gpio_param + kgpio_generic_param).c_str());
        }
        else {
            ROS_INFO("The following gpios have been provided as output pins:");

            for (int i = 0; i != _output_gpio_params_.size(); i++) {
                ROS_INFO("    GPIO[%i]", _output_gpio_params_.at(i));
            }
        }
        //=========================================================================================
        ParamStatus input_parameters_good  = rosParamCheck(
                                kconfig_sub_area + kinput_gpio_param + kgpio_generic_param,
                                _input_gpio_params_);

        if          (input_parameters_good == ParamStatus::kParameter_present_but_invalid) {
            ROS_ERROR("Input ROS Parameter %s was not set to the correct type. Expecting list "
                      "type to be provided. i.e. \"[21, 20, 16, 26]\"",
                      (kconfig_sub_area + kinput_gpio_param + kgpio_generic_param).c_str());
        }
        else if   (input_parameters_good == ParamStatus::kParameter_not_present) {
            ROS_WARN("Input ROS Parameter %s was not within roscore",
                    (kconfig_sub_area + kinput_gpio_param + kgpio_generic_param).c_str());
        }
        else {
            ROS_INFO("The following gpios have been provided as input pins:");

            for (int i = 0; i != _input_gpio_params_.size(); i++) {
                ROS_INFO("    GPIO[%i]", _input_gpio_params_.at(i));
            }
        }

        //=========================================================================================
        // Determine if the parameters provided are sufficient for node construction to complete
        //
        if          ( (output_parameters_good == ParamStatus::kParameter_present_but_invalid)
                ||    (input_parameters_good  == ParamStatus::kParameter_present_but_invalid) ) {
            // If there was one of the ROS parameters provided, but of the wrong type. Assume that
            // this 'should' be set - so need to get user to correct error. Return error
            ROS_ERROR("One of the inputs provided is of the wrong type, please either "
                      "correct or remove the parameter from the namespace: to allow node to "
                      " be generated");
            return -1;
        } else if   ( (output_parameters_good != ParamStatus::kParameter_present)
                &&    (input_parameters_good  != ParamStatus::kParameter_present) ) {
            ROS_ERROR("Expected ROS Parameters not within roscore, or not valid");
            return -1;
        }

        return 0;
    }

    /*
     *  @brief:  Separate function, to handle the hardware service callback queue.
     *           Intended to be used within a dedicated thread.
     *
     *  @param:  void
     *  @retval: void
     */
    //void hardwareCallbackThread(void) {
    //    ros::SingleThreadedSpinner spinner;
    //    spinner.spin(&_hardware_callback_queue_);
    //}

    /*
     *  @brief:  Function to encapsulate the looping of this node.
     *
     *  @param:  void
     *  @retval: void
     */
    void nodeLoop(void) {
        ROS_INFO("GPIO node ready for use");

        // Make use of custom hardware callback queue
        ros::SingleThreadedSpinner spinner;
        spinner.spin(&_hardware_callback_queue_);

        //ros::spin();
    }

    /*
     *  @brief:  Setups the GPIOs for the node, as per the expected input/configuration parameters
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
        // Input parameters
        if (checkGPIOInputParameters() < 0) {
            return -1;
        }

        _private_nh_.param<double>(kconfig_sub_area + knode_loop_rate,
                                   _loop_rate_parameter_,
                                   10);

        //=========================================================================================
        // Duplication check
        ROS_INFO("Checking that there is no overlap of gpio numbers...");
        if(duplicationParamCheck() < 0) {       // If duplication(s) detected
            return -1;                          // exit
        }
        ROS_INFO("No GPIO duplications detected");

        //=========================================================================================
        ROS_INFO("Generating the GPIOs");
        // Input parameters how now been checked, and any errors would of exitted before getting
        // here...
        GPIO::piSetup();        // Needed to make GPIO pins work

        // 'at' version of the std::vector used instead of the operator[], as this provides
        // detection of accessing something outside the bounds of the pinaddress.
        // Generate the array of 'output' GPIOs
        for (int i = 0; i != _output_gpio_params_.size(); i++) {
            _output_array_.push_back(   GPIO(   GPIO::State::kLow,
                                                _output_gpio_params_.at(i),
                                                GPIO::Dir::kOutput)  );

            ROS_INFO("Output --> GPIO[%i]\tconstructed as bit %i", _output_gpio_params_.at(i), i);
        }

        for (int i = 0; i != _input_gpio_params_.size(); i++) {
            _input_array_.push_back(    GPIO(   GPIO::State::kLow,
                                                _input_gpio_params_.at(i),
                                                GPIO::Dir::kInput)  );

            ROS_INFO("Input --> GPIO[%i]\tconstructed as bit %i", _input_gpio_params_.at(i), i);
        }

        ROS_INFO("GPIO has been setup");
        //=========================================================================================
        // Publishers/Subscribers
        if (_input_gpio_params_.size() != 0) {
        _gpio_input_publisher_  = _nh_.advertise<std_msgs::UInt64>(
                                                             kGPIO_publish_input_state,
                                                             20);
        }
        //=========================================================================================
        // Timers
        if (_input_gpio_params_.size() != 0) {
        _gpio_output_check_timer_ = _nh_hardware_.createTimer(
                                                        ros::Duration(1/_loop_rate_parameter_),
                                                        &rosGPIO::callbackGPIOget,
                                                        this,
                                                        false);

        }

        //=========================================================================================
        // Clients/Servers
        if (_output_gpio_params_.size() != 0) {
        _set_server_        = _nh_hardware_.advertiseService(kGPIO_set_service,
                                                             &rosGPIO::callbackGPIOset,
                                                             this);
        }
        //=========================================================================================

        ROS_INFO("GPIO Server constructed");
        return 0;   // If got to this point, no errors were detected
    }

    /*
     *  @brief:  Basic function, will take the input value, and configure the GPIOs within node
     *           to display that value - in binary
     *           Essentially a wrapper for the GPIO function - 'setGPIOArray'
     *
     *  @param:  unsigned 32bit value
     *  @retval: void
     */
    void setGPIOvalue(uint32_t new_value)
    {
        GPIO::setGPIOArray(_output_array_.data(), _output_array_.size(), new_value);
    }

    /*
     *  @brief:  Basic function, will take the return the value that is configured within the
     *           GPIOs of the node
     *           Essentially a wrapper for the GPIO function - 'setGPIOArray'
     *
     *  @retval: unsigned 32bit value
     */
    uint32_t getGPIOvalue()
    {
        return GPIO::getGPIOArray(_input_array_.data(), _input_array_.size());
    }

    /*
     *  @brief:  Callback function to be used as part of the service of this node. Will receive
     *           in the GPIOctrl message, to set the GPIOs constructed in this class to the desired
     *           value
     *
     *  @param:  milibrary GPIOCtrl request
     *  @param:  milibrary GPIOCtrl response (nothing)
     *  @retval: Service needs to return a boolean type
     */
    bool callbackGPIOset(milibrary::GPIOctrl::Request &req, milibrary::GPIOctrl::Response &res) {
        setGPIOvalue(req.value);
        return true;
    }

    /*
     *  @brief:  Callback function to be used to get the states of the input GPIOs.
     *           Rate of transmission is based upon the timer function which calls this
     *
     *  @param:  ROS Timer event
     *  @retval: None
     */
    void callbackGPIOget(const ros::TimerEvent& event) {
        std_msgs::UInt64 msg;

        msg.data = getGPIOvalue();
        _gpio_input_publisher_.publish(msg);
    }

    ~rosGPIO() {
        ROS_INFO("Shuting down node, and returning pins to FALSE");
        // Drive the GPIOs to a default value of "0"
        setGPIOvalue( 0 );
    }

};

//=================================================================================================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gpio_basic");
    ros::NodeHandle n;
    ros::NodeHandle private_params("~");

    rosGPIO  node(&n, &private_params);

    ros::shutdown();            // If get to this point of the main() function. Then should be
                                // shutdowning the node - due to error
    ros::waitForShutdown();

    // On node shutdown, don't think it reaches this part of main()
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

