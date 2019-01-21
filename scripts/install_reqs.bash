#!/bin/bash

set -e

echo ">[INSTALL]<START> Installing project requirements"

echo ">[INSTALL]<START> Installing from apt and pip"

sudo apt-get install \
	build-essential \
	cmake \
	unzip \
	git \
	libgtk2.0-dev \
	pkg-config \
	libavcodec-dev \
	libavformat-dev \
	libswscale-dev \
	qt5-default \
 	libqt5opengl5-dev \
	libgtk-3-dev \
	libgtkglext1 \
	libgtkglext1-dev \
	libtbb2 \
	libtbb-dev \
	libjpeg-dev \
	libpng-dev \
	libtiff-dev \
	libdc1394-22-dev


echo ">[INSTALL]<DONE> Installing from apt and pip"

echo ">[INSTALL]<DONE> Installing project requirements"
