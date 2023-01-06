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

#include "tensorrt_yolov7/node.hpp"

/* ----------------------------- BBox Includes ---------------------------- */
namespace perception
{
namespace tensorrt_yolov7_ros2
{

TensorRTYolov7Ros2Node::TensorRTYolov7Ros2Node(const rclcpp::NodeOptions & options)
:  rclcpp::Node("tensorrt_yolov7_ros2_node", options)
{
  // node->declare_parameter<std::string>("engine_path_");
  // node->declare_parameter<int>("debug");
  // node->declare_parameter<std::string>("frame_id");
  engine_path_ = declare_parameter<std::string>("engine_path");
  debug_ = declare_parameter<int>("debug");
  frame_id_ = declare_parameter<std::string>("frame_id");
  yolov7_ = std::make_unique<Yolov7>(engine_path_);
  bgr_imgs_ = std::make_shared<std::vector<cv::Mat>>();
  bgr_imgs_->reserve(1);

  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto sensor_msgs_qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

  // Creating Subscribers
  input_flc_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "vimba_front_left_center/image", sensor_msgs_qos,
      std::bind(&TensorRTYolov7Ros2Node::image_callback, this, std::placeholders::_1));
  input_frc_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "vimba_front_right_center/image", sensor_msgs_qos,
      std::bind(&TensorRTYolov7Ros2Node::image_callback, this, std::placeholders::_1));
  input_fl_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "vimba_front_left/image", sensor_msgs_qos,
      std::bind(&TensorRTYolov7Ros2Node::image_callback, this, std::placeholders::_1));
  input_fr_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "vimba_front_rear/image", sensor_msgs_qos,
      std::bind(&TensorRTYolov7Ros2Node::image_callback, this, std::placeholders::_1));
  input_rl_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "vimba_rear_left/image", sensor_msgs_qos,
      std::bind(&TensorRTYolov7Ros2Node::image_callback, this, std::placeholders::_1));
  input_rr_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "vimba_rear_right/image", sensor_msgs_qos,
      std::bind(&TensorRTYolov7Ros2Node::image_callback, this, std::placeholders::_1));
  
  // Creating Publishers
  detection_flc_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "vimba_front_left_center/out/image",
        sensor_msgs_qos
  );
    detection_frc_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "vimba_front_right_center/out/image",
        sensor_msgs_qos
  );
    detection_fl_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "vimba_front_left/out/image",
        sensor_msgs_qos
  );
    detection_fr_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "vimba_front_right/out/image",
        sensor_msgs_qos
  );
    detection_rl_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "vimba_rear_left/out/image",
        sensor_msgs_qos
  );
    detection_rr_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "vimba_rear_right/out/image",
        sensor_msgs_qos
  );
  
  objects_flc_pub_ = this->create_publisher<vision_msgs::msg::BoundingBox2D>(
        "vimba_front_left_center/out/objects",
        sensor_msgs_qos);
      objects_frc_pub_ = this->create_publisher<vision_msgs::msg::BoundingBox2D>(
        "vimba_front_right_center/out/objects",
        sensor_msgs_qos);
          objects_fl_pub_ = this->create_publisher<vision_msgs::msg::BoundingBox2D>(
        "vimba_front_left/out/objects",
        sensor_msgs_qos);
          objects_fr_pub_ = this->create_publisher<vision_msgs::msg::BoundingBox2D>(
        "vimba_front_right/out/objects",
        sensor_msgs_qos);
          objects_rl_pub_ = this->create_publisher<vision_msgs::msg::BoundingBox2D>(
        "vimba_rear_left/out/objects",
        sensor_msgs_qos);
          objects_rr_pub_ = this->create_publisher<vision_msgs::msg::BoundingBox2D>(
        "vimba_rear_right/out/objects",
        sensor_msgs_qos);
  // objects_pub_ = this->create_publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
  //       "vimba_front_left_center/out/objects",
  //       sensor_msgs_qos
  // );

  params_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&TensorRTYolov7Ros2Node::param_callback, this, std::placeholders::_1));
  
}

rcl_interfaces::msg::SetParametersResult TensorRTYolov7Ros2Node::param_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  for (const auto & param : parameters) {
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      if (param.get_name() == "debug") {
        debug_ = param.as_int();
        result.successful = true;
      }
    } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      if (param.get_name() == "frame_id") {
        frame_id_ = param.as_string();
        result.successful = true;
      }
    }
  }
  result.reason = result.successful ? "success" : "failure";
  return result;
}

void TensorRTYolov7Ros2Node::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // Get the Timestamp
  auto step_time = this->now();
  auto frame = 
  try {
    //
    cv_ptr_ = cv_bridge::toCvCopy(msg, msg->encoding);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // If the 
  if (bgr_imgs_->empty()) {
    bgr_imgs_->push_back(cv_ptr_->image);
  } else {
    bgr_imgs_->at(0) = cv_ptr_->image;
  }

  yolov7_->preProcess(*bgr_imgs_);

  yolov7_->infer();

  // nmsresults likely posess a list of bounding boxes
  nmsresults_ = yolov7_->PostProcess();

  // Create the Object Bboxes msg to publish
  // vison_msgs::msg::Detection2_d_array
  auto pub_msg = vision_msgs::msg::BoundingBox2D();

  for(size_t i =0; i < nmsresults_.size();i++){
      // TODO: Publish here!
      Yolov7::DrawBoxesonGraph(bgr_imgs_->at(i),nmsresults_[i]);

      // Only Publish the first box
      if (i == 0 and nmsresults_[i].size()>0) {
          auto& ibox = nmsresults_[i][i];
          float left = ibox[0];
          float top = ibox[1];
          float right = ibox[2];
          float bottom = ibox[3];
          int class_label = ibox[4];
          float confidence = ibox[5];

          pub_msg.center.x = int((right - left)/2 + left);
          pub_msg.center.y = int((bottom - top)/2 + top);
          pub_msg.size_x = int(right - left);
          pub_msg.size_y = int(bottom - top);
      }
    //   // Testing Print out the results
    //   for (int h = 0; h < nmsresults_.size(); h++) {
    //     for (int j = 0; j < nmsresults_[h].size(); j++) {
    //       for (int k = 0; k < nmsresults_[h][j].size(); k++) {
    //         std::cout << "i = " << i << ", j = " << j << ", k = " << k << ", Value = " << nmsresults_[h][j][k] <<std::endl;
    //       }
    //     }
    //   }
    //   std::cout << "Next" <<std::endl;
      
    //   // Publish the image and Bounding Boxes
    //   cv_ptr_->image = bgr_imgs_->at(i);
    //   detection_image_publisher_->publish(*(cv_ptr_->toImageMsg()).get() );
      if (nmsresults_[i].size()>0){objects_pub_->publish(pub_msg);}
  }

  auto stop = now();
  auto diff = stop - step_time;
  if (debug_ > 0) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), debug_,"TensorRT Yolov7 Node took %f seconds", diff.seconds());
  }
}

}    // namespace tensorrt_yolov7_ros2
}  // namespace perception

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options{};
  auto node = std::make_shared<perception::tensorrt_yolov7_ros2::TensorRTYolov7Ros2Node>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
// #elif
// #end
