#include "ros2_tensorrt.hpp"

Ros2TensorrtModule::Ros2TensorrtModule(const Ros2TensorrtModuleConfig &config)
    : Node(config.node_name),
      sub_topic_(config.sub_topic),
      pub_topic_(config.pub_topic),
      model_path_(config.model_path),
      batch_size_(config.batch_size),
      max_workspace_(config.max_workspace) {
  // handle the TensorRT module
  if (strcmp(tensorrt_common::getFileType(model_path_).c_str(), "onnx") == 0) {
    std::cout << "<STATUS> Building TensorRT Module from ONNX model now."
              << std::endl;
    tensorrt_module_ = std::make_unique<TensorRTModule>(
        model_path_, batch_size_, max_workspace_);
  } else {
    std::cout << "<STATUS> Building TensorRT Module from serialized TensorRT "
                 "Engine now."
              << std::endl;
    tensorrt_module_ =
        std::make_unique<TensorRTModule>(model_path_, batch_size_);
  }
  std::cout << "<STATUS> TensorRT Module ready for inference now." << std::endl;

  // save some info for safety checking
  auto input_dims = tensorrt_module_->get_input_dimensions(0);
  if (input_dims.nbDims == 3) {
    input_channels_ = static_cast<uint32_t>(input_dims.d[0]);
    input_rows_ = static_cast<uint32_t>(input_dims.d[1]);
    input_cols_ = static_cast<uint32_t>(input_dims.d[2]);
  } else
    throw std::runtime_error(
        "<ERROR> TensorRT Module input dimensions inappropriate.");

  // handle the ROS2 items
  array_subscriber_ =
      this->create_subscription<std_msgs::msg::Float32MultiArray>(
          sub_topic_, std::bind(&Ros2TensorrtModule::array_callback, this, _1));
  array_publisher_ =
      this->create_publisher<std_msgs::msg::Float32MultiArray>(pub_topic_);

  // handle information displaying
  info_timer_ = this->create_wall_timer(
      1000ms, std::bind(&Ros2TensorrtModule::info_timer_callback, this));

  std::cout << "<STATUS> ROS2 TensorRT Module initialized, ready for callback "
               "inference."
            << std::endl;
}

Ros2TensorrtModule::~Ros2TensorrtModule() {
  std::cout << "<STATUS> ROS2 TensorRT Module killed." << std::endl;
}

void Ros2TensorrtModule::array_callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  // safety for input sizes
  auto input_dims = tensorrt_module_->get_input_dimensions(0);
  int n_dims = 0;
  uint32_t n_elems = 1;
  for (auto dimension : msg->layout.dim) {
    if (dimension.size != input_dims.d[n_dims])
      throw std::runtime_error(
          "<ERROR> Incoming data dimension inconsistent with TensorRT Module.");
    n_elems *= dimension.size;
    n_dims++;
  }
  if (input_dims.nbDims != n_dims)
    throw std::runtime_error(
        "<ERROR> Incoming data incorrect number of dimensions");

  // prepare data in the form of vector<float>
  std::vector<float> input_data(&(msg->data[0]),
                                &(msg->data[0]) + static_cast<size_t>(n_elems));
  std::vector<std::vector<float>> input_buffer;
  input_buffer.push_back(input_data);

  // inference
  if (!tensorrt_module_->inference(input_buffer))
    std::cout << "<ERROR> Inference failed." << std::endl;

  // Create the output data message
  std_msgs::msg::Float32MultiArray output_msg;
  auto output_dims = tensorrt_module_->get_output_dimensions(0);
  for (int dim_ind = 0; dim_ind < output_dims.nbDims; dim_ind++) {
    std_msgs::msg::MultiArrayDimension curr_dim;
    curr_dim.size = static_cast<uint32_t>(output_dims.d[dim_ind]);
    output_msg.layout.dim.push_back(curr_dim);
  }

  // Publish the inference results
  std::vector<float> output = tensorrt_module_->get_output(0);
  output_msg.data.insert(output_msg.data.end(), output.begin(), output.end());
  array_publisher_->publish(output_msg);

  // For rate keeping
  n_messages_++;
}

void Ros2TensorrtModule::info_timer_callback() {
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "<STATUS> Inference Rate: "
            << static_cast<float>(1000.0 / n_messages_) << "Hz" << std::endl;
  n_messages_ = 0;
}