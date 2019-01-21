#!/bin/bash

set -e

# Print the full directory path of the root of this project
function print_project_root {
  readlink -f `dirname $(readlink -f "$0")`/../..
}

cd `print_project_root`

echo ">[BUILD]<START> Downloading Eigen library..."

TARGET_DIR=3rd_party/eigen
BRANCH=branches/3.3

mkdir -p $TARGET_DIR

git clone -b $BRANCH https://github.com/eigenteam/eigen-git-mirror.git $TARGET_DIR

echo ">[BUILD]<DONE> Downloading Eigen library..."
