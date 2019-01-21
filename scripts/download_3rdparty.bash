#!/bin/bash

set -e

# Print the full directory path of the root of this project
function print_project_root {
  readlink -f `dirname $(readlink -f "$0")`/..
}

cd `print_project_root`

DOWNLOAD_SCRIPT_GLOB="scripts/3rd_party/*.bash"

echo ">[BUILD]<START>< Downloading third party using scripts: $DOWNLOAD_SCRIPT_GLOB"

for f in scripts/3rd_party/*.bash; do
  echo ">[BUILD]<START>< Running script: $f"
  bash "$f" -H || break;
  echo ">[BUILD]<DONE>< Running script: $f"
done

echo ">[BUILD]<DONE>< Downloading third party using scripts: $DOWNLOAD_SCRIPT_GLOB"