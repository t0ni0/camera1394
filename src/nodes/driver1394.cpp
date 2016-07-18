/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2009, 2010 Jack O'Quin, Patrick Beeson
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <boost/format.hpp>
#include <sstream>  // debug

#include <driver_base/SensorLevels.h>
#include <tf/transform_listener.h>

#include "driver1394.h"
#include "camera1394/Camera1394Config.h"
#include "features.h"

/** @file

@brief ROS driver for IIDC-compatible IEEE 1394 digital cameras.

This is a ROS driver for 1394 cameras, using libdc1394.  It can be
instantiated as either a node or a nodelet.  It is written with with
minimal dependencies, intended to fill a role in the ROS image
pipeline similar to the other ROS camera drivers.

@par Advertises

 - @b camera/image_raw topic (sensor_msgs/Image) raw 2D camera images

 - @b camera/camera_info topic (sensor_msgs/CameraInfo) Calibration
   information for each image.

*/

namespace camera1394_driver
{
  // some convenience typedefs
  typedef camera1394::Camera1394Config Config;
  typedef driver_base::Driver Driver;
  typedef driver_base::SensorLevels Levels;

  Camera1394Driver::Camera1394Driver(ros::NodeHandle priv_nh,
                                     ros::NodeHandle camera_nh):
    vio_thread_alive_(false),
    state_(Driver::CLOSED),
    reconfiguring_(false),
    priv_nh_(priv_nh),
    camera_nh_(camera_nh),
    camera_name_("camera"),
    cycle_(1.0),                        // slow poll when closed
    retries_(0),
    ros_frame_count_(0),
    dev_(new camera1394::Camera1394()),
    srv_(priv_nh),
    cinfo_(new camera_info_manager::CameraInfoManager(camera_nh_)),
    calibration_matches_(true),
    it_(new image_transport::ImageTransport(camera_nh_)),
    image_pub_(it_->advertiseCamera("image_raw", 10)),
    ros_trig_wait_pub_(camera_nh.advertise<std_msgs::Int16>("waiting_trigger", 100)),
    ros_timestamp_sub_(camera_nh.subscribe("/mavros/cam_imu_sync/cam_imu_stamp",
        1, &Camera1394Driver::bufferTimestamp, this)),
    get_camera_registers_srv_(camera_nh_.advertiseService(
                                "get_camera_registers",
                                &Camera1394Driver::getCameraRegisters, this)),
    set_camera_registers_srv_(camera_nh_.advertiseService(
                                "set_camera_registers",
                                &Camera1394Driver::setCameraRegisters, this)),
    trigger_ready_srv_(camera_nh_.serviceClient<std_srvs::Trigger>("trigger_ready")),
    diagnostics_(),
    topic_diagnostics_min_freq_(0.),
    topic_diagnostics_max_freq_(1000.),
    topic_diagnostics_("image_raw", diagnostics_,
               diagnostic_updater::FrequencyStatusParam
               (&topic_diagnostics_min_freq_,
            &topic_diagnostics_max_freq_, 0.1, 10),
               diagnostic_updater::TimeStampStatusParam())
  {
    image_buffer_.reserve(10);
    cinfo_buffer_.reserve(10);
    timestamp_buffer_.reserve(10);
  }

  Camera1394Driver::~Camera1394Driver()
  {}

  /** Close camera device
   *
   *  postcondition: state_ is Driver::CLOSED
   */
  void Camera1394Driver::closeCamera()
  {
    if (state_ != Driver::CLOSED)
      {
        ROS_INFO_STREAM("[" << camera_name_ << "] closing device");
        dev_->close();
        state_ = Driver::CLOSED;
      }
  }

  /** Open the camera device.
   *
   * @param newconfig configuration parameters
   * @return true, if successful
   *
   * @post diagnostics frequency parameters set
   *
   * if successful:
   *   state_ is Driver::OPENED
   *   camera_name_ set to GUID string
   *   GUID configuration parameter updated
   */
  bool Camera1394Driver::openCamera(Config &newconfig)
  {
    bool success = false;

    try
      {
        if (0 == dev_->open(newconfig))
          {
            if (camera_name_ != dev_->device_id_)
              {
                camera_name_ = dev_->device_id_;
                if (!cinfo_->setCameraName(camera_name_))
                  {
                    // GUID is 16 hex digits, which should be valid.
                    // If not, use it for log messages anyway.
                    ROS_WARN_STREAM("[" << camera_name_
                                    << "] name not valid"
                                    << " for camera_info_manger");
                  }
              }
            ROS_INFO_STREAM("[" << camera_name_ << "] opened: "
                            << newconfig.video_mode << ", "
                            << newconfig.frame_rate << " fps, "
                            << newconfig.iso_speed << " Mb/s");
            state_ = Driver::OPENED;
            calibration_matches_ = true;
            newconfig.guid = camera_name_; // update configuration parameter
            retries_ = 0;
            success = true;
          }
      }
    catch (camera1394::Exception& e)
      {
        state_ = Driver::CLOSED;    // since the open() failed
        if (retries_++ > 0)
          ROS_DEBUG_STREAM("[" << camera_name_
                           << "] exception opening device (retrying): "
                           << e.what());
        else
          ROS_ERROR_STREAM("[" << camera_name_
                           << "] device open failed: " << e.what());
      }

    // update diagnostics parameters
    diagnostics_.setHardwareID(camera_name_);
    double delta = newconfig.frame_rate * 0.1; // allow 10% error margin
    topic_diagnostics_min_freq_ = newconfig.frame_rate - delta;
    topic_diagnostics_max_freq_ = newconfig.frame_rate + delta;

    return success;
  }


  /** device poll */
  void Camera1394Driver::poll(void)
  {
    // Do not run concurrently with reconfig().
    //
    // The mutex lock should be sufficient, but the Linux pthreads
    // implementation does not guarantee fairness, and the reconfig()
    // callback thread generally suffers from lock starvation for many
    // seconds before getting to run.  So, we avoid acquiring the lock
    // if there is a reconfig() pending.
    bool do_sleep = true;
    if (!reconfiguring_)
      {
        boost::mutex::scoped_lock lock(mutex_);
        if (state_ == Driver::CLOSED)
          {
            openCamera(config_);        // open with current configuration
          }
        do_sleep = (state_ == Driver::CLOSED);
        if (!do_sleep)                  // openCamera() succeeded?
          {
            // driver is open, read the next image still holding lock
            sensor_msgs::ImagePtr image(new sensor_msgs::Image);
            if (read(image))
              {
                publish(image);
              }
          }
      } // release mutex lock

    // Always run the diagnostics updater: no lock required.
    diagnostics_.update();

    if (do_sleep)
      {
        // device was closed or poll is not running, sleeping avoids
        // busy wait (DO NOT hold the lock while sleeping)
        cycle_.sleep();
      }
  }

  /** Publish camera stream topics
   *
   *  @param image points to latest camera frame
   */
  void Camera1394Driver::publish(const sensor_msgs::ImagePtr &image)
  {
    image->header.frame_id = config_.frame_id;

    // get current CameraInfo data
    sensor_msgs::CameraInfoPtr
      ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));

    // check whether CameraInfo matches current video mode
    if (!dev_->checkCameraInfo(*image, *ci))
      {
        // image size does not match: publish a matching uncalibrated
        // CameraInfo instead
        if (calibration_matches_)
          {
            // warn user once
            calibration_matches_ = false;
            ROS_WARN_STREAM("[" << camera_name_
                            << "] calibration does not match video mode "
                            << "(publishing uncalibrated data)");
          }
        ci.reset(new sensor_msgs::CameraInfo());
        ci->height = image->height;
        ci->width = image->width;
      }
    else if (!calibration_matches_)
      {
        // calibration OK now
        calibration_matches_ = true;
        ROS_WARN_STREAM("[" << camera_name_
                        << "] calibration matches video mode now");
      }

    // fill in operational parameters
    dev_->setOperationalParameters(*ci);

    ci->header.frame_id = config_.frame_id;
    ci->header.stamp = image->header.stamp;
    ci->header.seq = image->header.seq = ros_frame_count_++;

    // Publish via image_transport
    //image_pub_.publish(image, ci);

    // Publishing is disabled here because the trigger sync thread will
    // do that for us. Instead we buffer the image and in camera info
    // for the sync thread to use.
    image_buffer_.push_back(image);
    cinfo_buffer_.push_back(ci);

    if(image_buffer_.size() > 5) image_buffer_.erase(image_buffer_.begin());
    if(cinfo_buffer_.size() > 5) cinfo_buffer_.erase(cinfo_buffer_.begin());


    // Notify diagnostics that a message has been published. That will
    // generate a warning if messages are not published at nearly the
    // configured frame_rate.
    topic_diagnostics_.tick(image->header.stamp);
  }

  /** Read camera data.
   *
   * @param image points to camera Image message
   * @return true if successful, with image filled in
   */
  bool Camera1394Driver::read(sensor_msgs::ImagePtr &image)
  {
    bool success = true;
    try
      {
        // Read data from the Camera
        ROS_DEBUG_STREAM("[" << camera_name_ << "] reading data");
        success = dev_->readData(*image);
        ROS_DEBUG_STREAM("[" << camera_name_ << "] read returned");
      }
    catch (camera1394::Exception& e)
      {
        ROS_WARN_STREAM("[" << camera_name_
                        << "] Exception reading data: " << e.what());
        success = false;
      }
    return success;
  }

  /** Dynamic reconfigure callback
   *
   *  Called immediately when callback first defined. Called again
   *  when dynamic reconfigure starts or changes a parameter value.
   *
   *  @param newconfig new Config values
   *  @param level bit-wise OR of reconfiguration levels for all
   *               changed parameters (0xffffffff on initial call)
   **/
  void Camera1394Driver::reconfig(Config &newconfig, uint32_t level)
  {
    // Do not run concurrently with poll().  Tell it to stop running,
    // and wait on the lock until it does.
    reconfiguring_ = true;
    boost::mutex::scoped_lock lock(mutex_);
    ROS_DEBUG("dynamic reconfigure level 0x%x", level);

    // resolve frame ID using tf_prefix parameter
    if (newconfig.frame_id == "")
      newconfig.frame_id = "camera";
    std::string tf_prefix = tf::getPrefixParam(priv_nh_);
    ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
    newconfig.frame_id = tf::resolve(tf_prefix, newconfig.frame_id);

    if (state_ != Driver::CLOSED && (level & Levels::RECONFIGURE_CLOSE))
      {
        // must close the device before updating these parameters
        closeCamera();                  // state_ --> CLOSED
      }

    if (state_ == Driver::CLOSED)
      {
        // open with new values
        openCamera(newconfig);
      }

    if (config_.camera_info_url != newconfig.camera_info_url)
      {
        // set the new URL and load CameraInfo (if any) from it
        if (cinfo_->validateURL(newconfig.camera_info_url))
          {
            cinfo_->loadCameraInfo(newconfig.camera_info_url);
          }
        else
          {
            // new URL not valid, use the old one
            newconfig.camera_info_url = config_.camera_info_url;
          }
      }

    if (state_ != Driver::CLOSED)       // openCamera() succeeded?
      {
        // configure IIDC features
        if (level & Levels::RECONFIGURE_CLOSE)
          {
            // initialize all features for newly opened device
            if (false == dev_->features_->initialize(&newconfig))
              {
                ROS_ERROR_STREAM("[" << camera_name_
                                 << "] feature initialization failure");
                closeCamera();          // can't continue
              }
          }
        else
          {
            // update any features that changed
            // TODO replace this with a direct call to
            //   Feature::reconfigure(&newconfig);
            dev_->features_->reconfigure(&newconfig);
          }
      }

    config_ = newconfig;                // save new parameters
    reconfiguring_ = false;             // let poll() run again

    ROS_DEBUG_STREAM("[" << camera_name_
                     << "] reconfigured: frame_id " << newconfig.frame_id
                     << ", camera_info_url " << newconfig.camera_info_url);
  }


  /** driver initialization
   *
   *  Define dynamic reconfigure callback, which gets called
   *  immediately with level 0xffffffff.  The reconfig() method will
   *  set initial parameter values, then open the device if it can.
   */
  void Camera1394Driver::setup(void)
  {
    srv_.setCallback(boost::bind(&Camera1394Driver::reconfig, this, _1, _2));

    sendTriggerWaiting(config_.exposure);

    startFramePublisher();

    sendTriggerReady();

  }


  /** driver termination */
  void Camera1394Driver::shutdown(void)
  {
    closeCamera();
    stopFramePublisher();
  }

  /** Callback for getting camera control and status registers (CSR) */
  bool Camera1394Driver::getCameraRegisters(
      camera1394::GetCameraRegisters::Request &request,
      camera1394::GetCameraRegisters::Response &response)
  {
    typedef camera1394::GetCameraRegisters::Request Request;
    boost::mutex::scoped_lock lock(mutex_);
    if (state_ == Driver::CLOSED)
      {
        return false;
      }
    if (request.num_regs < 1
        || (request.type != Request::TYPE_CONTROL
            && request.type != Request::TYPE_ADVANCED_CONTROL))
      {
        request.num_regs = 1;
      }
    response.value.resize(request.num_regs);

    bool success = false;
    switch (request.type)
      {
      case Request::TYPE_CONTROL:
        success = dev_->registers_->getControlRegisters(
              request.offset, request.num_regs, response.value);
        break;
      case Request::TYPE_ABSOLUTE:
        success = dev_->registers_->getAbsoluteRegister(
              request.offset, request.mode, response.value[0]);
        break;
      case Request::TYPE_FORMAT7:
        success = dev_->registers_->getFormat7Register(
              request.offset, request.mode, response.value[0]);
        break;
      case Request::TYPE_ADVANCED_CONTROL:
        success = dev_->registers_->getAdvancedControlRegisters(
              request.offset, request.num_regs, response.value);
        break;
      case Request::TYPE_PIO:
        success = dev_->registers_->getPIORegister(
              request.offset, response.value[0]);
        break;
      case Request::TYPE_SIO:
        success = dev_->registers_->getSIORegister(
              request.offset, response.value[0]);
        break;
      case Request::TYPE_STROBE:
        success = dev_->registers_->getStrobeRegister(
              request.offset, response.value[0]);
        break;
      }

    if (!success)
      {
        ROS_WARN("[%s] getting register failed: type %u, offset %lu",
                 camera_name_.c_str(), request.type, request.offset);
      }
    return success;
  }

  /** Callback for setting camera control and status registers (CSR) */
  bool Camera1394Driver::setCameraRegisters(
      camera1394::SetCameraRegisters::Request &request,
      camera1394::SetCameraRegisters::Response &response)
  {
    typedef camera1394::SetCameraRegisters::Request Request;
    if (request.value.size() == 0)
      return true;
    boost::mutex::scoped_lock lock(mutex_);
    if (state_ == Driver::CLOSED)
      return false;
    bool success = false;
    switch (request.type)
      {
      case Request::TYPE_CONTROL:
        success = dev_->registers_->setControlRegisters(
              request.offset, request.value);
        break;
      case Request::TYPE_ABSOLUTE:
        success = dev_->registers_->setAbsoluteRegister(
              request.offset, request.mode, request.value[0]);
        break;
      case Request::TYPE_FORMAT7:
        success = dev_->registers_->setFormat7Register(
              request.offset, request.mode, request.value[0]);
        break;
      case Request::TYPE_ADVANCED_CONTROL:
        success = dev_->registers_->setAdvancedControlRegisters(
              request.offset, request.value);
        break;
      case Request::TYPE_PIO:
        success = dev_->registers_->setPIORegister(
              request.offset, request.value[0]);
        break;
      case Request::TYPE_SIO:
        success = dev_->registers_->setSIORegister(
              request.offset, request.value[0]);
        break;
      case Request::TYPE_STROBE:
        success = dev_->registers_->setStrobeRegister(
              request.offset, request.value[0]);
        break;
      }

    if (!success)
      {
        ROS_WARN("[%s] setting register failed: type %u, offset %lu",
                 camera_name_.c_str(), request.type, request.offset);
      }
    return success;
  }

  /**
   * @brief Buffers a CamIMUStamp coming from the autopilot through mavros.
   * @param msg The timestamp to buffer
   */
  void Camera1394Driver::bufferTimestamp(const mavros_msgs::CamIMUStamp& msg)
  {
    timestamp_buffer_.push_back(msg);

    // TODO Add half of exposure time
    // XXX check for dropped timestamp msg

    // Check if buffer exceeds limit size
    // Throw away oldest image if it does
    if(timestamp_buffer_.size() > 5){
        timestamp_buffer_.erase(timestamp_buffer_.begin());
    }

  }

  /**
   * @brief Inform the system that the camera driver (this node) is done
   * setting up and that the camera is ready to be triggered.
   */
  void Camera1394Driver::sendTriggerReady()
  {
    std_srvs::Trigger sig;
    trigger_ready_srv_.waitForExistence();
    if(!trigger_ready_srv_.call(sig)){
        ROS_ERROR("Failed to call ready-for-trigger on service: %s",
                    trigger_ready_srv_.getService().c_str() );
    }
  }

  /**
   * @brief
   * @param exposure_ms
   */
  void Camera1394Driver::sendTriggerWaiting(int exposure_ms)
  {
      std_msgs::Int16 exposure_ready;
      exposure_ready.data = exposure_ms;

      ros_trig_wait_pub_.publish(exposure_ready);
  }

  /**
   * @brief The function which will be run in a seperate thread to publish
   * correctly timestamped images aligned with IMU data.
   */
  void Camera1394Driver::framePublishLoop()
  {
    //ROS_INFO("frame publish looped");
    ros::Rate idleDelay(200);

    while( ros::ok() && vio_thread_alive_ ){
      // check if buffers are not empty
      if(image_buffer_.size() && timestamp_buffer_.size()){
          for(unsigned int i = 0; i < image_buffer_.size() && timestamp_buffer_.size() > 0;) {
              // If this fails it return 1 which will advance the pointer i.
              // If this succeeds, i can stay the same value and will point to
              // a new element since the published ones will be removed.
              i += stampAndPublishImage(i);
          }
      }
      idleDelay.sleep();
    }
  }

  /**
   * @brief Start the frame-imu synced image publishing loop (thread)
   */
  void Camera1394Driver::startFramePublisher()
  {
    vio_thread_alive_ = true;
    vio_pub_thread_ = std::thread(std::bind(&Camera1394Driver::framePublishLoop,
                                            this));
  }

  /**
   * @brief Stop the frame-imu synce image publishing loop (thread)
   */
  void Camera1394Driver::stopFramePublisher()
  {
    vio_thread_alive_ = false;
    if(vio_pub_thread_.joinable()) {
        vio_pub_thread_.join();
    }
    vio_pub_thread_ = std::thread();
  }

  /**
   * @brief Change the timestamp of an image to the one sent in by
   * mavros and then publish it.
   *
   * The new timestamp should be one which is matched with an IMU message.
   * @param Index in the buffers of the image to be published.
   * @return 0 on success, 1 on failure
   */
  unsigned int Camera1394Driver::stampAndPublishImage(unsigned int index)
  {
    // find the index of the imu timestamp corresponding to the image we
    // want to publish
    int timestamp_index = findInImgBuffer(index);

    // if we found the appropriate timestamp
    if (timestamp_index) {
        // Copy corresponding image and time stamps
        sensor_msgs::ImagePtr image;
        sensor_msgs::CameraInfoPtr ci;
        image = image_buffer_.at(index);
        ci = cinfo_buffer_.at(index);

        // Stamp it
        image->header.stamp = timestamp_buffer_.at(timestamp_index).frame_stamp;
        ci->header = image->header;   // Stamp the cam info with the same stamp

        // Publish image in ROS
        image_pub_.publish(image, ci);

        // Erase the stuff we just published
        image_buffer_.erase(image_buffer_.begin() + index);
        cinfo_buffer_.erase(cinfo_buffer_.begin() + index);
        timestamp_buffer_.erase(timestamp_buffer_.begin() + timestamp_index);
        return 0;
    } else {
        // Failure
        //ROS_WARN("findInImgBuffer failed");
        return 1;
    }
  }

  /**
   * @brief Find the IMU time stamp which corresponds to the image pointed by
   * the index.
   *
   * This is done by matching sequence numbers.
   *
   * @param index in the image buffer of the image who's time stamp we want to
   * find
   * @return 0 if we can't find it, or the index of the timestamp in the
   * timestamp buffer
   */
  unsigned int Camera1394Driver::findInImgBuffer(unsigned int index)
  {
    // Check if we have at least one image in the buffer
    if(image_buffer_.size() < 1){
        return 0;
    }

    unsigned int k = 0;
    while(k < timestamp_buffer_.size() && ros::ok()){
        // look for the matching sequence number
        // if (index != 0) ROS_INFO("Index: %d\tImage buffer seq: %d\tTimestamp buffer seq: %d", index, image_buffer_.at(index)->header.seq, timestamp_buffer_.at(k).frame_seq_id);
        if(image_buffer_.at(index)->header.seq ==
                (uint)timestamp_buffer_.at(k).frame_seq_id) {
            return k;
        } else {
            k += 1;
        }
    }

    //failure handler
//    std::ostringstream time_seq, img_seq;
//    for(int i = 0; i < timestamp_buffer_.size(); ++i){
//      time_seq << timestamp_buffer_.at(i).frame_seq_id << ' ';
//    }
//
//    for(int i = 0; i < image_buffer_.size(); ++i){
//      img_seq << image_buffer_.at(i)->header.seq << ' ';
//    }

    //ROS_INFO("findInImgBuffer seq numbers %s ~ %s", time_seq.str().c_str(), img_seq.str().c_str());

    return 0;
  }

}; // end namespace camera1394_driver
