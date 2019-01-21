#!/bin/bash

echo ">[CLEAN_DEPS]<START>< Cleaning built artifacts and built dependencies..."
rm -rf build_files
rm -rf bin
rm -rf deps/opencv/build/
rm -rf 3rd_party/*
echo ">[CLEAN_DEPS]<DONE>< Cleaning built artifacts and built dependencies..."
