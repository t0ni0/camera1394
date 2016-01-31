
#include "ros/ros.h"
#include "mavros_msgs/CommandTriggerControl.h"
#include <cstdlib>
#include <string>
#include <std_srvs/Trigger.h>
#include <std_msgs/Int16.h>

class TriggerReady
{

public:
    /**
     * @brief Default constructor
     */
    TriggerReady()
    {
        cam0_OK_ = false;
        cam1_OK_ = false;
        exposure_ms_ = 0;
        framerate_ = 15; //** in Hz, default framerate
        triggerClient_ = n_.serviceClient<mavros_msgs::CommandTriggerControl>("/mavros/cmd/trigger_control");
        advertiseService();
        subscribeCameras();
    }

    /**
     * @brief cam0Ready
     * @param msg
     */
    void cam0Ready(const std_msgs::Int16ConstPtr &msg)
    {
        exposure_ms_ = msg->data; // Set exposure from cam0
        if(exposure_ms_ > 1000/framerate_){
            ROS_WARN("Exposure time %u ms does not allow %u Hz framerate.",
                     exposure_ms_, framerate_);
        }
        ROS_INFO("Camera 0 waiting for trigger. Exposure set to %u ms", exposure_ms_);
    }

    /**
     * @brief cam1Ready
     * @param msg
     */
    void cam1Ready(const std_msgs::Int16ConstPtr &msg)
    {
        ROS_INFO("Camera 1 waiting for trigger. Exposure set to %u ms", exposure_ms_); //XXX check if the exposures aren't same.
    }

    /**
     * @brief Subscribe to the trigger waiting topics. These topics receive an
     * Int16 which is the exposure in milliseconds requested for the next trigger
     * pulse.
     */
    void subscribeCameras()
    {
        cam0_Sub_ = n_.subscribe("cam0/waiting_trigger", 0, &TriggerReady::cam0Ready, this);
        cam1_Sub_ = n_.subscribe("cam1/waiting_trigger", 0, &TriggerReady::cam1Ready, this);
    }

    /**
     * @brief Callback for cam0 trigger ready service
     *        Mark the cam0 as ready to be triggered
     * @param req
     * @param resp
     * @return Service
     */
    bool servCam0(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
    {
        cam0_OK_ = true;
        resp.success = true;
        ROS_INFO_STREAM("Camera 0 is primed for trigger");
        return true;
    }

    /**
     * @brief Callback for cam1 trigger ready service
     *        Mark the cam1 as ready to be triggered
     * @param req
     * @param resp
     * @return
     */
    bool servCam1(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
    {
        cam1_OK_ = true;
        resp.success = true;
        ROS_INFO_STREAM("Camera 1 is primed for trigger");
        return true;
    }

    /**
     * @brief Tells you if cam0 is ready for triggering
     * @return
     */
    bool cam0_OK()
    {
        return cam0_OK_;
    }

    /**
     * @brief Tells you if cam1 is ready for triggering
     * @return
     */
    bool cam1_OK()
    {
        return cam1_OK_;
    }

    /**
     * @brief Call the mavros service to trigger the cameras
     * @return 0 on success, 1 on failure
     */
    int sendTriggerCommand()
    {
        srv_.request.integration_time = (1000/framerate_);
        srv_.request.trigger_enable = true;

        if (triggerClient_.call(srv_)) {
            ROS_INFO("Successfully called service trigger_control");

        } else {
            ROS_ERROR("Failed to call service trigger_control");
            return 1;
        }

        return 0;
    }

    /**
     * @brief Advertise the trigger ready services. These services should be
     * called when the cameras have been set up and are ready to be triggered.
     */
    void advertiseService()
    {
        serverCam0_ = n_.advertiseService("cam0/camera/trigger_ready", &TriggerReady::servCam0, this);
        serverCam1_ = n_.advertiseService("cam1/camera/trigger_ready", &TriggerReady::servCam1, this);
    }


private:

    bool cam0_OK_;
    bool cam1_OK_;
    int exposure_ms_;
    int framerate_;

    ros::NodeHandle n_;

    ros::Subscriber cam0_Sub_;
    ros::Subscriber cam1_Sub_;

    ros::ServiceClient triggerClient_;
    mavros_msgs::CommandTriggerControl srv_;

    ros::ServiceServer serverCam0_;
    ros::ServiceServer serverCam1_;

};


/**
 * @brief This application will spin while waiting for the two camera drives to
 * come up. Once they are alive, the trigger command is sent at a rate of 10Hz
 * through mavros.
 * @param argc ros argument count
 * @param argv ros arguments
 * @return error code
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "StartTrigger");
    TriggerReady tr;

    // Define time update rate to call callback function if necessary
    ros::Rate r(100); // Hz

    while (!(tr.cam0_OK() && tr.cam1_OK()) && ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

    // Send start trigger command to Pixhawk
    ros::Rate r2(10); // Hz

    while (tr.sendTriggerCommand() && ros::ok()) {
        ROS_INFO_STREAM("Retrying reaching pixhawk");
        r2.sleep();
    }

}
