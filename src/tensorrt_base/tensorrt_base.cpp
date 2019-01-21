#include "tensorrt_base.hpp"

TensorRTModule::TensorRTModule(std::string model_path, int batch_size,
                               int max_workspace_size)
    : model_path_(model_path),
      model_type_(tensorrt_common::getFileType(model_path)),
      batch_size_(batch_size),
      max_workspace_size_(max_workspace_size) {
  // TODO: DLA stuff as mentioned in commons, probably play with fp16

  // create the initial objects needed
  trt_builder_ =
      tensorrt_common::infer_object(nvinfer1::createInferBuilder(g_logger_));
  trt_builder_->setMaxBatchSize(batch_size_);
  trt_builder_->setMaxWorkspaceSize(max_workspace_size_);
  // trt_builder and trt_runtime
  trt_network_ = tensorrt_common::infer_object(trt_builder_->createNetwork());
  trt_parser_ = tensorrt_common::infer_object(
      nvonnxparser::createParser(*trt_network_, g_logger_));
  trt_runtime_ =
      tensorrt_common::infer_object(nvinfer1::createInferRuntime(g_logger_));

  // load the onnx model
  if (strcmp(model_type_.c_str(), "onnx") == 0) {
    if (!trt_parser_->parseFromFile(model_path_.c_str(), verbosity_))
      runtime_error_("failed to parse onnx file");
  }

  // get some network details used in inference later
  n_inputs_ = trt_network_->getNbInputs();
  n_outputs_ = trt_network_->getNbOutputs();
  n_layers_ = trt_network_->getNbLayers();
  n_bindings_ = n_inputs_ + n_outputs_;
  input_dims_.reserve(n_inputs_);
  input_vols_.reserve(n_inputs_);
  output_dims_.reserve(n_outputs_);
  output_vols_.reserve(n_outputs_);
  engine_smart_mem_.reserve(n_bindings_);
  engine_mem_.reserve(n_bindings_);
  output_mem_.reserve(n_outputs_);
  output_mem_.clear();

  // saves the dimensions for use later, also mallocs on host and device
  for (int input_index = 0; input_index < n_inputs_; input_index++) {
    nvinfer1::Dims curr_dim =
        trt_network_->getInput(input_index)->getDimensions();
    input_dims_.push_back(curr_dim);
    input_vols_.push_back(tensorrt_common::volume(curr_dim));
    engine_smart_mem_.push_back(cuda_malloc_from_dims(curr_dim));
    engine_mem_.push_back(engine_smart_mem_.back().get());
  }
  for (int output_index = 0; output_index < n_outputs_; output_index++) {
    nvinfer1::Dims curr_dim =
        trt_network_->getOutput(output_index)->getDimensions();
    output_dims_.push_back(curr_dim);
    output_vols_.push_back(tensorrt_common::volume(curr_dim));
    engine_smart_mem_.push_back(cuda_malloc_from_dims(curr_dim));
    engine_mem_.push_back(engine_smart_mem_.back().get());

    std::vector<float> curr_output_mem(
        static_cast<int>(tensorrt_common::volume(curr_dim)));
    output_mem_.push_back(curr_output_mem);
  }

  // get the final engines, inference contexts and cudaStream
  trt_engine_ = tensorrt_common::infer_object(
      trt_builder_->buildCudaEngine(*trt_network_));
  if (n_bindings_ != trt_engine_->getNbBindings())
    runtime_error_("number of bindings inconsistent");
  trt_context_ =
      tensorrt_common::infer_object(trt_engine_->createExecutionContext());
  CHECK(cudaStreamCreate(&stream_));

  std::cout << "<STATUS> TensorRT module initialization done." << std::endl;
  std::cout << "<STATUS> Inference can be started now." << std::endl;
}

TensorRTModule::TensorRTModule(std::string model_path)
    : model_path_(model_path),
      model_type_(tensorrt_common::getFileType(model_path)) {
  trt_runtime_ =
      tensorrt_common::infer_object(nvinfer1::createInferRuntime(g_logger_));

  std::stringstream gie_model_stream;
  gie_model_stream.seekg(0, gie_model_stream.beg);
  std::ifstream cache(model_path_);
  gie_model_stream << cache.rdbuf();
  cache.close();

  gie_model_stream.seekg(0, std::ios::end);
  const int model_size = gie_model_stream.tellg();
  gie_model_stream.seekg(0, std::ios::beg);

  std::unique_ptr<char> model_mem(static_cast<char*>(malloc(model_size)));
  gie_model_stream.read(static_cast<char*>(model_mem.get()), model_size);
  trt_engine_ =
      tensorrt_common::infer_object(trt_runtime_->deserializeCudaEngine(
          model_mem.get(), (std::size_t)model_size, NULL));
  trt_context_ =
      tensorrt_common::infer_object(trt_engine_->createExecutionContext());
  CHECK(cudaStreamCreate(&stream_));

  // get some network details used in inference later, using engine
  n_layers_ = trt_engine_->getNbLayers();
  n_bindings_ = trt_engine_->getNbBindings();
  engine_smart_mem_.reserve(n_bindings_);
  engine_mem_.reserve(n_bindings_);

  n_inputs_ = 0;
  n_outputs_ = 0;
  input_dims_.clear();
  input_vols_.clear();
  output_dims_.clear();
  output_vols_.clear();
  output_mem_.clear();
  for (int binding = 0; binding < n_bindings_; binding++) {
    nvinfer1::Dims curr_dim = trt_engine_->getBindingDimensions(binding);
    if (trt_engine_->bindingIsInput(binding)) {
      n_inputs_++;
      input_dims_.push_back(curr_dim);
      input_vols_.push_back(tensorrt_common::volume(curr_dim));
    } else {
      n_outputs_++;
      output_dims_.push_back(curr_dim);
      output_vols_.push_back(tensorrt_common::volume(curr_dim));

      std::vector<float> curr_output_mem(
          static_cast<int>(tensorrt_common::volume(curr_dim)));
      output_mem_.push_back(curr_output_mem);
    }
    engine_smart_mem_.push_back(cuda_malloc_from_dims(curr_dim));
    engine_mem_.push_back(engine_smart_mem_.back().get());
  }

  std::cout << "<STATUS> TensorRT module initialization done." << std::endl;
  std::cout << "<STATUS> Inference can be started now." << std::endl;
}

TensorRTModule::~TensorRTModule() {
  cudaStreamDestroy(stream_);
  std::cout << "TensorRT module killed safely." << std::endl;
}

bool TensorRTModule::inference(const std::vector<std::vector<float>>& input) {
  // safety: must be same number of inputs
  if (static_cast<int>(input.size()) != n_inputs_)
    runtime_error_("number of inputs incorrect");

  // copy from inputs to device asynchronously
  for (int i = 0; i < n_inputs_; i++) {
    if (static_cast<int>(input[i].size() != input_vols_[i]))
      runtime_error_("input [" + std::to_string(i) + "] size incorrect");
    CHECK(cudaMemcpyAsync(engine_mem_[i], &input[i][0],
                          batch_size_ * input_vols_[i] * sizeof(float),
                          cudaMemcpyHostToDevice, stream_));
  }
  cudaStreamSynchronize(stream_);

  // inference
  this->trt_context_->enqueue(batch_size_, engine_mem_.data(), stream_,
                              nullptr);
  cudaStreamSynchronize(stream_);

  // copy from device to output asynchronously
  // engine memory pointers are concatenated as [input | output]
  for (int i = 0; i < n_outputs_; i++) {
    CHECK(cudaMemcpyAsync(&output_mem_[i][0], engine_mem_[i + n_inputs_],
                          batch_size_ * output_vols_[i] * sizeof(float),
                          cudaMemcpyDeviceToHost, stream_));
  }
  return true;
}

const std::vector<float>& TensorRTModule::get_output(int output_index) {
  if (output_index < 0 || output_index >= n_outputs_)
    runtime_error_("output index out of range, requesting " +
                   std::to_string(output_index) + " when there are " +
                   std::to_string(n_outputs_) + " outputs");
  return output_mem_[output_index];
}

void TensorRTModule::save_engine(std::string engine_path) {
  if (!trt_engine_) runtime_error_("Engine not created yet, unable to save.");

  std::shared_ptr<nvinfer1::IHostMemory> serialized_model =
      tensorrt_common::infer_object(trt_engine_->serialize());
  std::ofstream ofs(engine_path, std::ios::out | std::ios::binary);
  ofs.write((char*)(serialized_model->data()), serialized_model->size());
  ofs.close();
  std::cout << "<STATUS> TensorRT engine saved at path " << engine_path
            << std::endl;
}

void TensorRTModule::load_engine() {
  std::cout << "<STATUS> TensorRT engine loading done." << std::endl;
}

void print_dims(nvinfer1::Dims dimensions) {
  for (int i = 0; i < dimensions.nbDims; i++) {
    std::cout << dimensions.d[i] << " ";
  }
  std::cout << std::endl;
}