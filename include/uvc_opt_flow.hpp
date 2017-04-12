/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
*  uvc_opt_flow.hpp
*
*  Created on: April 05, 2017
*      Author: Christoph
*/


#ifndef __UVC_OPT_FLOW_H__
#define __UVC_OPT_FLOW_H__

#include <iostream>
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
// #include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>

#include "flow_opencv.hpp"

#define OPTICAL_FLOW_OUTPUT_RATE 15
#define CROP_SIZE_DEFAULT 128

class OptFlow {

public:

  OptFlow();
  ~OptFlow();

private:

  ros::NodeHandle nh_;
  ros::Subscriber sub_image_raw;
  ros::Subscriber sub_image_depth;
  ros::Subscriber sub_imu;
  ros::Subscriber sub_cam_info;

  std::string uvc_node_name_;

  bool calibration_initialized;
  float focal_length[2];
  float principal_point[2];
  uint32_t image_size[2];
  float radial_distortion[3];
  float base_line;

  bool reset_gyro;
  float gyro_x_int;
  float gyro_y_int;
  float gyro_z_int;

  OpticalFlowOpenCV *_optical_flow;

  void imageRawCb(const sensor_msgs::ImageConstPtr& msg);
  void imageDepthCb(const sensor_msgs::ImageConstPtr& msg);
  void imuCb(const sensor_msgs::ImuConstPtr& msg);
  void cameraInfoCb(const sensor_msgs::CameraInfo::ConstPtr& msg);
  void send_mavlink_message(const uint8_t msgid, const void *msg, uint8_t component_ID);

};

#endif /* end of include guard: __UVC_OPT_FLOW_H__ */
