# Open Source TensorRT Wrapper

## To do:

- ros2_base
  - create both float32 messages and JPEG messages
  - try using libjpegturbo to encode and decode instead of OpenCV
  - make retrieval of output easier
  - add post processing plugin
- ros_base

## Tested and rolling with:

- Cmake 3.10.2
- gcc 7.3.0
- TensorRT 5.0.2.6
- CUDA 10
- GTX1050 Ti
- ROS2 Crystal

## Third-party Prerequisites (installation difficulty):

- TensorRT
- CUDA
- ROS2

## Building, flags needed

```
git clone --recursive https://github.com/aaronchongth/ros2_tensorrt.git
./build_deps
./build -DTENSORRT_DIR [***]
```

## Cleaning, rebuilding

```
./clean
./build -DTENSORRT_DIR [***]
```

## Cleaning all, including built dependencies, rebuilding

```
./clean_deps
./build_deps
./build -DTENSORRT_DIR [***]
```

## Example

Major examples and tests would be using [pretrained models](https://github.com/Cadene/pretrained-models.pytorch)

```

```