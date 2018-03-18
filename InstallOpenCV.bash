#!/usr/bin/env bash
# Written by Dominic Cupo on 5-19-2017
# Using source from Manuel Ignacio López Quintero @ https://github.com/milq/milq/blob/master/scripts/bash/install-opencv.sh

# Modified to also install opencv_contrib modules
# Also fixes some additional problems I ran into

#Makes sure that CMake is up to 3.x levels. Alleviates issues with bundled CMake not being high enough
sudo apt-get install software-properties-common
sudo add-apt-repository ppa:george-edison55/cmake-3.x


# Make sure other distro packages are up to date

sudo apt-get -y update
sudo apt-get -y upgrade


# INSTALL THE DEPENDENCIES

# Build tools:
sudo apt-get install -y build-essential cmake

# GUI:
sudo apt-get install -y qt5-default libvtk6-dev

# Media I/O:
sudo apt-get install -y zlib1g-dev libjpeg-dev libwebp-dev libpng-dev libtiff5-dev libjasper-dev libopenexr-dev libgdal-dev

# Video I/O:
sudo apt-get install -y libdc1394-22-dev libavcodec-dev libavformat-dev libswscale-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev yasm libopencore-amrnb-dev libopencore-amrwb-dev libv4l-dev libxine2-dev

# Parallelism and linear algebra libraries:
sudo apt-get install -y libtbb-dev libeigen3-dev

# Python:
sudo apt-get install -y python-dev python-tk python-numpy python3-dev python3-tk python3-numpy

# Java:
sudo apt-get install -y ant default-jdk

# Documentation:
sudo apt-get install -y doxygen


# Install helpful script tools
sudo apt-get install -y unzip wget

#Download, unzip, rename OpenCV Core
wget https://github.com/opencv/opencv/archive/master.zip
unzip master.zip
rm master.zip
mv opencv-master OpenCV

#Download, unzip, rename OpenCV Modules
wget https://github.com/opencv/opencv_contrib/archive/master.zip
unzip master.zip
rm master.zip
mv opencv_contrib-master OpenCV_Contrib

#Create build dir, then build it all!
cd OpenCV
mkdir build
cd build
cmake -DENABLE_PRECOMPILED_HEADERS=OFF -DWITH_QT=ON -DWITH_OPENGL=ON -DFORCE_VTK=ON -DWITH_TBB=ON -DWITH_GDAL=ON -DWITH_XINE=ON -DBUILD_EXAMPLES=OFF -DBUILD_DOCS=OFF  -G "Eclipse CDT4 - Unix Makefiles" -DOPENCV_EXTRA_MODULES_PATH=../../OpenCV_Contrib/modules ..
make -j4
sudo make install
sudo ldconfig
cd ../..

rm -rf OpenCV_Contrib OpenCV
