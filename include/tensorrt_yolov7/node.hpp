// Copyright 2022 Siddharth Saha
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License

/// \copyright Copyright 2022 Siddharth Saha
/// \file
/// \brief This file defines the tensorrt_yolov7_ros2_node class.

#ifndef TENSORRT_YOLOV7_ROS2__TENSORRT_YOLOV7_ROS2_NODE_HPP_
#define TENSORRT_YOLOV7_ROS2__TENSORRT_YOLOV7_ROS2_NODE_HPP_

/* ------------------------ Standard Library Includes ----------------------- */
#include <chrono>
#include <memory>
#include <vector>
#include <numeric>
#include <random>
#include <string>

/* ------------------------------ ROS2 Includes ----------------------------- */
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
// #include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/bounding_box2_d.hpp"
#include "tier4_perception_msgs/msg/detected_objects_with_feature.hpp"

/* ----------------------- TensorRT & YOLOv7 Includes ----------------------- */
#include "tensorrt_yolov7/Yolov7.h"

/* ----------------------------- OpenCV Includes ---------------------------- */
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "cv_bridge/cv_bridge.h"

namespace perception
{
namespace tensorrt_yolov7_ros2
{

class TensorRTYolov7Ros2Node : public rclcpp::Node
{
public:
  explicit TensorRTYolov7Ros2Node(const rclcpp::NodeOptions & options);

  rcl_interfaces::msg::SetParametersResult param_callback(
    const std::vector<rclcpp::Parameter> & parameters);

private:

  std::string frame_id_;
  std::string engine_path_;
  std::unique_ptr<Yolov7> yolov7_;
  std::shared_ptr<std::vector<cv::Mat>> bgr_imgs_;
  std::vector<std::vector<std::vector<float>>> nmsresults_;
  cv_bridge::CvImagePtr cv_ptr_;
  int debug_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr input_image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
    detection_image_publisher_;
  // rclcpp::Publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr objects_pub_;
  rclcpp::Publisher<vision_msgs::msg::BoundingBox2D>::SharedPtr objects_pub_;

  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
};
}    // namespace tensorrt_yolov7_ros2
}  // namespace perception

#endif  // TENSORRT_YOLOV7_ROS2__TENSORRT_YOLOV7_ROS2_NODE_HPP_
