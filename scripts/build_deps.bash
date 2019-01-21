#!/bin/bash

set -e

# Print the full directory path of the root of this project
function print_project_root {
  readlink -f `dirname $(readlink -f "$0")`/..
}

cd `print_project_root`

echo ">[BUILD]<START>< Building OpenCV"
./scripts/build_opencv.bash
echo ">[BUILD]<DONE>< Building OpenCV"

echo ">[BUILD]<START>< Downloading third party libraries"
./scripts/download_3rdparty.bash
echo ">[BUILD]<DONE>< Downloading third party libraries"

