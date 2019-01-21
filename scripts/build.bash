#!/bin/bash

set -e

# Print the full directory path of the root of this project
function print_project_root {
  readlink -f `dirname $(readlink -f "$0")`/..
}

BUILD_DIRECTORY=build_files
BUILD_MESSAGE=" Building project..."

function run_cmake {
  echo ">[BUILD]<START>< Running cmake..."
  pushd build_files >/dev/null
  cmake ..
  popd >/dev/null
  echo ">[BUILD]<DONE>< Running cmake..."
}

function run_make {
  echo ">[BUILD]<START>< Running make..."
  pushd build_files >/dev/null
  make
  popd >/dev/null
  echo ">[BUILD]<DONE>< Running make..."
}

echo ">[BUILD]<START>< $BUILD_MESSAGE"

# Ensure we're in the project's root directory
cd `print_project_root`

if [ ! -d "$BUILD_DIRECTORY" ]; then
  mkdir -p $BUILD_DIRECTORY
fi

run_cmake
run_make

echo ">[BUILD]<DONE>< $BUILD_MESSAGE"
