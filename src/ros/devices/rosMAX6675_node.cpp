/**************************************************************************************************
 * @file        rosMAX6675_node.cpp
 * @author      Thomas
 * @brief       ROS node for the MAX6675 class
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * This node is called "max6675_node"
 *   Configuration parameters:
 *      ~/config        /loop_rate
 *                      [ Floating data value, specifying the rate that the Linux device is
 *                      [ interrogated.
 *                      [ - Note, if none is provided will default to '4Hz'
 *
 *                      /address
 *                      [ The SPI address for the MAX6675 device
 *
 *   Publishers:
 *      Temperature
 *                      [ Provides the external temperature reading read from the MAX6675 device
 *                      [ Uses sensor_msgs/msg/Temperature.msg
 *
 *   Subscribers:
 *      None
 *
 *   Services:
 *      None
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

#include <sensor_msgs/Temperature.h>

// Project Libraries
// -----------------
#include "milibrary/BUSctrl.h"

#include "FileIndex.h"
// ~~~~~~~~~~~~~~~~~~~
#include FilInd_MAX6675HD               // Header for MAX6675 class

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
class rosLnx {
private:
    std::string         kconfig_sub_area            = "config/";

    std::string         knode_loop_rate             = "loop_rate";
    std::string         knode_spi_address           = "address";

    std::string         kMAX6675_publish_temp       = "Temperature";

    // Inputs
    std::string         kSPI_transfer_service       = "transfer_spi";

private:
    MAX6675                 *_hardware_handle_  = NULL;

/**************************************************************************************************
 * == ROS STUFF == >>>   ROBOT OPERATING SYSTEM (ROS) OBJECTS    <<<
 *   -----------
 *  The follow are objects used for interfacing with the Robot Operating System.
 *************************************************************************************************/
    ros::NodeHandle         _nh_;
    ros::NodeHandle         _private_nh_;

    ros::NodeHandle         _nh_hardware_;
    ros::CallbackQueue      _hardware_callback_queue_;

    // PARAMETERS
    ////////////////////////
    double                          _loop_rate_parameter_;
    double                          _spi_address_;

    // MESSAGES
    ////////////////////////
    sensor_msgs::Temperature        _max6675_temperature_message_;

    // PUBLISHERS
    ////////////////////////
    ros::Publisher                  _max6675_temperature_publisher_;

    // SUBSCRIBERS
    ////////////////////////

    // TIMERS
    ////////////////////////

    // SERVICES
    ////////////////////////
    ros::ServiceClient              _spi_transfer_client_;

    // ACTIONS
    ////////////////////////

public:
/*
 *  @brief:  Create the MAX6675 node class, basic construction
 *
 *  @param:  Pointer to the 'NodeHandle'
 *  @param:  Pointer to the 'NodeHandle', setup for private - ("~")
 *  @retval: rosLnx class
 */
    rosLnx(ros::NodeHandle* normal, ros::NodeHandle* private_namespace):
    _nh_(*normal), _private_nh_(*private_namespace)
    {
        //_nh_hardware_.setCallbackQueue(&_hardware_callback_queue_);

        if (configNode() < 0) {
            ROS_ERROR("Error detected during MAX6675 construction, exiting node...");
            return;
        }

        nodeLoop();
    }

    /*
     *  @brief:  Function to call up the spi transfer client to send the packet(s) of data to
     *           MAX6675
     *           Generates a blank 2 byte array to faciliate the SPI transfer (MAX6675 has no MOSI
     *           input)
     *
     *  @param:  void
     *  @retval: void
     */
    void callSPITransferClient(void) {
        milibrary::BUSctrl          msg;

        // Clear the data initially
        msg.request.address   = _spi_address_;
        msg.request.read_size = 0;
        msg.request.write_data.insert(msg.request.write_data.begin(), 2, 0);

        // Push data to the SPI service
        _spi_transfer_client_.call(msg);

        _hardware_handle_->poleTempRead(msg.response.read_data.data());
    }

    /*
     *  @brief:  Separate function, to handle the hardware service callback queue.
     *           Intended to be used within a dedicated thread.
     *
     *  @param:  void
     *  @retval: void
     */
    void hardwareCallbackThread(void) {
        ros::SingleThreadedSpinner spinner;
        spinner.spin(&_hardware_callback_queue_);
    }

    /*
     *  @brief:  Function to encapsulate the looping of this node.
     *
     *  @param:  void
     *  @retval: void
     */
    void nodeLoop(void) {
        ROS_INFO("MAX6675 node ready for use");

        ros::Rate loop_rate(_loop_rate_parameter_);

        _max6675_temperature_message_.temperature   = -999;
        _max6675_temperature_message_.variance      = 0;

        _max6675_temperature_message_.header.seq    = 0;

        while (ros::ok()) {
            _max6675_temperature_message_.header.stamp  = ros::Time::now();

            // Setup SPI transfer message
            callSPITransferClient();
            _max6675_temperature_message_.temperature = _hardware_handle_->temp;

            _max6675_temperature_publisher_.publish(_max6675_temperature_message_);

            ros::spinOnce();
            loop_rate.sleep();
            _max6675_temperature_message_.header.seq++;
        }
    }

    /*
     *  @brief:  Setups the LnxCond for the node, as per the expected input/configuration
     *           parameters from within the rosparam space.
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
        _private_nh_.param<double>(kconfig_sub_area + knode_loop_rate,
                                   _loop_rate_parameter_,
                                   4.0);

        _private_nh_.param<double>(kconfig_sub_area + knode_spi_address,
                                   _spi_address_,
                                   0.0);
        //=========================================================================================
        // Duplication check

        //=========================================================================================

        _hardware_handle_ = new MAX6675();

        ROS_INFO("MAX6675 has been setup");
        //=========================================================================================
        // Publishers
        _max6675_temperature_publisher_ = _nh_.advertise<sensor_msgs::Temperature>(
                                                kMAX6675_publish_temp,
                                                5);

        //=========================================================================================
        // Timers

        //=========================================================================================
        // Clients/Servers
        for (uint8_t i = 0; i != 10; i++) {
            if ( !(ros::service::exists(kSPI_transfer_service, false) ) )  {
                ROS_WARN("Required services are not currently available...pause count %d", i);
                ros::Duration(1).sleep(); // sleep for a second.
            }
            else {
                break;
            }

            // On last iteration
            if (i == 9) {
                ROS_ERROR("Timed out waiting for the services to be setup, shutdowning node...");
                return -1;
            }
        }

        _spi_transfer_client_   = _nh_.serviceClient<milibrary::BUSctrl>(
                                                kSPI_transfer_service,
                                                true);

        //=========================================================================================

        ROS_INFO("MAX6675 node constructed");
        return 0;   // If got to this point, no errors were detected
    }

    ~rosLnx() {
        ROS_INFO("Shutting down the node, and killing functions");
        delete _hardware_handle_;
    }

};

//=================================================================================================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "max6675_basic");
    ros::NodeHandle n;
    ros::NodeHandle private_params("~");

    rosLnx  node(&n, &private_params);

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

