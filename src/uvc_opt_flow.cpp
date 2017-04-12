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
*  uvc_opt_flow.cpp
*
*  Created on: April 05, 2017
*      Author: Christoph
*/

#include "uvc_opt_flow.hpp"

#include "mavlink_types.h"
#include "common/mavlink.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

OptFlow::OptFlow():
      nh_("~"),
      uvc_node_name_("/uvc_camera"),
      calibration_initialized(false),
      reset_gyro(true)
{
  // subscribe to topics
  printf("Subscribing to topics...\n");
  sub_image_raw = nh_.subscribe(uvc_node_name_ + "/cam_0/image_raw", 10, &OptFlow::imageRawCb, this);
  sub_image_depth = nh_.subscribe(uvc_node_name_ + "/cam_0/image_depth", 10, &OptFlow::imageDepthCb, this);
  sub_cam_info = nh_.subscribe(uvc_node_name_ + "/cam_0/camera_info", 1, &OptFlow::cameraInfoCb, this);
  sub_imu = nh_.subscribe(uvc_node_name_ + "/cam_0/imu", 100, &OptFlow::imuCb, this);

}

OptFlow::~OptFlow(){
  ROS_INFO("terminating");
}

void OptFlow::imageRawCb(const sensor_msgs::ImageConstPtr& msg) {

  if (!calibration_initialized)
    return;

  static mavlink_optical_flow_rad_t optflow_msg;
  float ang_flow_x = 0.0f;
  float ang_flow_y = 0.0f;
  static int dt_us = 0;
  uint32_t image_timestamp_us = msg->header.stamp.sec * 1e6 + msg->header.stamp.nsec / 1e3;

  //TODO check if possible without cv_bridge/OpenCV
  //convert and crop the image
  static cv::Rect crop(image_size[1]/2-CROP_SIZE_DEFAULT/2, image_size[0]/2-CROP_SIZE_DEFAULT/2,
      CROP_SIZE_DEFAULT, CROP_SIZE_DEFAULT);
  cv::Mat img = cv_bridge::toCvCopy(msg, "mono8")->image;
  cv::Mat croppedImage = img(crop);

  int flow_quality = _optical_flow->calcFlow(croppedImage.data, image_timestamp_us, dt_us, ang_flow_x, ang_flow_y);

  //check if flow should be published (in general, frame rate differs from publish rate)
  if (flow_quality >= 0) {

    // printf("test raw dt_us = %d   flow_x = %f   flow_y =  %f\n", dt_us, ang_flow_x, ang_flow_y);
    optflow_msg.time_usec = image_timestamp_us;
		optflow_msg.sensor_id = 0; //?
		optflow_msg.integration_time_us = dt_us;
		optflow_msg.integrated_x = ang_flow_x;
		optflow_msg.integrated_y = ang_flow_y;
    optflow_msg.integrated_xgyro = gyro_x_int / (dt_us / 1e6);
    optflow_msg.integrated_ygyro = gyro_y_int / (dt_us / 1e6);
    optflow_msg.integrated_zgyro = gyro_z_int / (dt_us / 1e6);
		optflow_msg.temperature = 0.0;
		optflow_msg.quality = flow_quality;
		optflow_msg.time_delta_distance_us = 0.0; //?
		optflow_msg.distance = -1.0; // mark as invalid

    //send optical flow mavlink message to px4
		send_mavlink_message(MAVLINK_MSG_ID_OPTICAL_FLOW_RAD, &optflow_msg, 200);

    reset_gyro = true;
  }

}

void OptFlow::imageDepthCb(const sensor_msgs::ImageConstPtr& msg) {

  if (!calibration_initialized)
    return;

  //convert and crop the image
  static cv::Rect crop(image_size[1]/2-CROP_SIZE_DEFAULT/2, image_size[0]/2-CROP_SIZE_DEFAULT/2,
      CROP_SIZE_DEFAULT, CROP_SIZE_DEFAULT);
  cv::Mat img = cv_bridge::toCvCopy(msg, "mono8")->image;
  cv::Mat croppedImage = img(crop);

  //average TODO histogram? outlier rejection?
  float depth_sum = cv::sum(croppedImage)[0];
  int non_zero = cv::countNonZero(croppedImage);

  float average_depth = depth_sum / non_zero;
  float disparity = average_depth/8.0f;

  //calculate distance from disparity
  if (disparity > 0) {
    float depth_m = focal_length[0] * base_line / disparity;

    //TODO lowpassfilter?
    printf("distance = %f\n", depth_m);
  } else {
    ROS_WARN("Disparity <= 0.0");
  }

}

void OptFlow::imuCb(const sensor_msgs::ImuConstPtr& msg) {

  if (!calibration_initialized)
    return;

  // reset gyro integration for new flow message
  if (reset_gyro) {
    gyro_x_int = 0.0f;
    gyro_y_int = 0.0f;
    gyro_z_int = 0.0f;

    reset_gyro = false;
  }

  float imu_timestamp = msg->header.stamp.sec + msg->header.stamp.nsec / 1e9;
  static float imu_timestamp_old = imu_timestamp;
  float imu_dt = imu_timestamp - imu_timestamp_old;

  // integrate gyro
  gyro_x_int += msg->angular_velocity.x * imu_dt;
  gyro_y_int += msg->angular_velocity.y * imu_dt;
  gyro_z_int += msg->angular_velocity.z * imu_dt;

  imu_timestamp_old = imu_timestamp;

}

void OptFlow::cameraInfoCb(const sensor_msgs::CameraInfo::ConstPtr& msg) {

  if (calibration_initialized)
    return;

  //set camera parameters
  image_size[0] = msg->width;
  image_size[1] = msg->height;
  focal_length[0] = msg->K[0];
  focal_length[1] = msg->K[4];
  principal_point[0] = msg->K[2];
  principal_point[1] = msg->K[5];
  base_line = 0.06f;//sqrt(msg->P[3]*msg->P[3] + msg->P[7]*msg->P[7]);

  //TODO creates a crash -> check in uvc
  radial_distortion[0] = 0.0f;//msg->D[0];
  radial_distortion[1] = 0.0f;//msg->D[1];
  radial_distortion[2] = 0.0f;//msg->D[4];

  if (image_size[0] <= 0 || image_size[1] <= 0 || focal_length[0] <= 0.0f || focal_length[0] <= 0.0f ||
      base_line <= 0.0f) {
    ROS_ERROR("Calibration issue: image size, focal length or base line is invalid");
    ros::shutdown();
  }

  //initialize optical flow
	_optical_flow = new OpticalFlowOpenCV(focal_length[0], focal_length[1], OPTICAL_FLOW_OUTPUT_RATE,
      CROP_SIZE_DEFAULT, CROP_SIZE_DEFAULT);
	_optical_flow->setCameraMatrix(focal_length[0], focal_length[1], principal_point[0], principal_point[1]);
	_optical_flow->setCameraDistortion(radial_distortion[0], radial_distortion[1], radial_distortion[2]);

  calibration_initialized = true;

  ROS_INFO("camera calibration initialized\n");
}

void OptFlow::send_mavlink_message(const uint8_t msgid, const void *msg, uint8_t component_ID) {

}
