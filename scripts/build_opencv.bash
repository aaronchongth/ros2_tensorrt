#!/bin/bash

set -e

# Print the full directory path of the root of this project
function print_project_root {
  readlink -f `dirname $(readlink -f "$0")`/..
}

cd `print_project_root`

echo ">[BUILD]<START>< Building OpenCV"

OPENCV_DIR=`pwd`/deps/opencv
OPENCV_CONTRIB_DIR=`pwd`/deps/opencv_contrib
OPENCV_TARGET_VERSION=3.4.0

mkdir -p $OPENCV_DIR
mkdir -p $OPENCV_CONTRIB_DIR

# Must be compiled using the same compiler as Nvidia

# Checkout opencv
if [ ! -f "$OPENCV_DIR/CMakeLists.txt" ]; then
  echo ">[BUILD]<START>< Downloading OpenCV"
  git clone https://github.com/opencv/opencv.git $OPENCV_DIR
  pushd $OPENCV_DIR >/dev/null
  git fetch --tags
  git checkout $OPENCV_TARGET_VERSION
  popd >/dev/null
  echo ">[BUILD]<DONE>< Downloading OpenCV"
fi

if [ ! -f "$OPENCV_CONTRIB_DIR/README.md" ]; then
  echo ">[BUILD]<START>< Downloading OpenCV contrib"
  git clone https://github.com/opencv/opencv_contrib.git $OPENCV_CONTRIB_DIR
  pushd $OPENCV_CONTRIB_DIR >/dev/null
  git fetch --tags
  git checkout $OPENCV_TARGET_VERSION
  popd >/dev/null
  echo ">[BUILD]<DONE>< Downloading OpenCV contrib"
fi

# Build opencv
# Important flags for cross compiling: 
# - CMAKE_TOOLCHAIN_FILE: path to toolchain file which comes with opencv
# - CXX_FLAGS: need to prepend -L/usr/local/cuda/targets/aarch64-linux/lib to use aarch64 cuda libs
# - CMAKE_C/CXX_COMPILER: use the nvidia compilers, rather than system.

echo ">[BUILD]<START>< Running OpenCV cmake"

pushd $OPENCV_DIR >/dev/null
mkdir -p build
pushd build >/dev/null

cmake \
    -DBUILD_SHARED_LIBS=OFF \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr \
    -DWITH_CUDA=OFF \
    -DINSTALL_C_EXAMPLES=OFF \
    -DINSTALL_TESTS=OFF \
    -DOPENCV_EXTRA_MODULES_PATH=$OPENCV_CONTRIB_DIR/modules \
    -DBUILD_LIST=features2d,xfeatures2d,highgui,imgcodecs,imgprocs,calib3d,videoio,video \
    ..

echo ">[BUILD]<DONE>< Running OpenCV cmake"

echo ">[BUILD]<START>< Running OpenCV make"
make -j6
echo ">[BUILD]<DONE>< Running OpenCV make"

popd >/dev/null
popd >/dev/null

echo ">[BUILD]<DONE>< Building OpenCV for cross-compilation"
