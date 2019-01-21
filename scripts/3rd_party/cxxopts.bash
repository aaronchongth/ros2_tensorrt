#!/bin/bash

set -e

# Print the full directory path of the root of this project
function print_project_root {
  readlink -f `dirname $(readlink -f "$0")`/../..
}

cd `print_project_root`

echo ">[BUILD]<START> Downloading cxxopts library..."

TARGET_DIR=3rd_party/cxxopts/include/cxxopts

mkdir -p $TARGET_DIR

wget https://raw.githubusercontent.com/jarro2783/cxxopts/e725ea308468ab50751ba7f930842a4c061226e9/include/cxxopts.hpp -P $TARGET_DIR

echo ">[BUILD]<DONE> Downloading cxxopts library..."
