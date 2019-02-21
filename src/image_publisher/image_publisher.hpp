#pragma once

// STL
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// OpenCV
#include "opencv2/opencv.hpp"

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
using namespace std::chrono_literals;

// CXXOPTS
#include "cxxopts/cxxopts.hpp"
namespace opt = cxxopts;

// Input Normalization
#define RED_CHANNEL_MEAN 0.485
#define RED_CHANNEL_STD 0.229
#define GREEN_CHANNEL_MEAN 0.456
#define GREEN_CHANNEL_STD 0.224
#define BLUE_CHANNEL_MEAN 0.406
#define BLUE_CHANNEL_STD 0.225

struct ImagePublisherConfig {
  std::string image_path = "data/cat_224.jpg";
  std::string node_name = "cat_pub";
  std::string jpg_topic = "jpg_topic";
  std::string array_topic = "array_topic";
};

class ImagePublisher : public rclcpp::Node {
 public:
  explicit ImagePublisher(const ImagePublisherConfig &config)
      : Node(config.node_name),
        image_path_(config.image_path),
        jpg_topic_(config.jpg_topic),
        array_topic_(config.array_topic) {
    // create the ROS2 items
    jpg_publisher_ =
        this->create_publisher<sensor_msgs::msg::CompressedImage>(jpg_topic_);
    array_publisher_ =
        this->create_publisher<std_msgs::msg::Float32MultiArray>(array_topic_);
    timer_ = this->create_wall_timer(
        25ms, std::bind(&ImagePublisher::timer_callback, this));

    // preparing the image for publishing
    cv::Mat img = cv::imread(image_path_);
    cv::imencode(".jpg", img, jpg_buf_);

    // preparing the float array for publishing
    cv::Mat float_mat;
    img.convertTo(float_mat, CV_32FC3, 1.0 / 255.0);

    std::vector<cv::Mat> img_channels;
    cv::split(float_mat, img_channels);
    img_channels[0] = img_channels[0] - BLUE_CHANNEL_MEAN;
    img_channels[0] = img_channels[0] / BLUE_CHANNEL_STD;
    img_channels[1] = img_channels[1] - GREEN_CHANNEL_MEAN;
    img_channels[1] = img_channels[1] / GREEN_CHANNEL_STD;
    img_channels[2] = img_channels[2] - RED_CHANNEL_MEAN;
    img_channels[2] = img_channels[2] / RED_CHANNEL_STD;

    cv::Mat img_vect_mat;
    img_vect_mat.push_back(img_channels[2]);
    img_vect_mat.push_back(img_channels[1]);
    img_vect_mat.push_back(img_channels[0]);
    img_vect_mat = img_vect_mat.reshape(1, 1);
    float_buf_.assign((float *)(img_vect_mat.datastart),
                      (float *)(img_vect_mat.dataend));

    array_msg_.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    array_msg_.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    array_msg_.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    array_msg_.layout.dim[0].label = "Channel";
    array_msg_.layout.dim[0].size = static_cast<uint32_t>(3);
    array_msg_.layout.dim[0].stride =
        static_cast<uint32_t>(img.rows * img.cols);
    array_msg_.layout.dim[1].label = "Height";
    array_msg_.layout.dim[1].size = static_cast<uint32_t>(img.rows);
    array_msg_.layout.dim[1].stride = static_cast<uint32_t>(img.cols);
    array_msg_.layout.dim[2].label = "Width";
    array_msg_.layout.dim[2].size = static_cast<uint32_t>(img.cols);
    array_msg_.layout.dim[2].stride = static_cast<uint32_t>(1);
    array_msg_.data.clear();
    array_msg_.data.insert(array_msg_.data.end(), float_buf_.begin(),
                           float_buf_.end());

    std::cout << "<STATUS> Image publisher initialized." << std::endl;
  }

  ~ImagePublisher() {
    std::cout << "<STATUS> Image publisher exited cleanly." << std::endl;
  }

 private:
  std::string image_path_, jpg_topic_, array_topic_;
  uint32_t seq_num_ = 0;
  rclcpp::TimerBase::SharedPtr timer_;

  // Publishing CompressedImage (JPEG)
  sensor_msgs::msg::CompressedImage jpg_msg_;
  std::vector<uchar> jpg_buf_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr
      jpg_publisher_;

  // Publishing Float32MultiArray
  std_msgs::msg::Float32MultiArray array_msg_;
  std::vector<float> float_buf_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      array_publisher_;

  void timer_callback() {
    seq_num_++;

    // handle CompressedImage publisher
    jpg_msg_.header.frame_id = "/img_frame";
    auto t_now = std::chrono::system_clock::now();
    auto duration = t_now.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
    duration -= seconds;
    auto nanoseconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(duration);

    jpg_msg_.header.stamp.sec = (int)(seconds.count());
    jpg_msg_.header.stamp.nanosec = (int)(nanoseconds.count());
    jpg_msg_.format = "jpeg";
    jpg_msg_.data = jpg_buf_;
    jpg_publisher_->publish(jpg_msg_);

    // handle Float32MultiArray publisher
    array_publisher_->publish(array_msg_);

    if (seq_num_ % 40 == 0)
      std::cout << "<STATUS> Publishing: Float32MultiArray "
                   "message at 25.00 Hz."
                << std::endl;
  }
};