#!/bin/bash
set -e

echo "=================================="
echo "Building OpenCV 4.10 with contrib"
echo "=================================="

cd /workspace

# Clone OpenCV if not present
if [ ! -d "opencv" ]; then
  echo "Cloning OpenCV 4.10.0..."
  git clone --branch 4.10.0 --depth 1 https://github.com/opencv/opencv.git
else
  echo "OpenCV source already exists"
fi

# Clone OpenCV contrib if not present
if [ ! -d "opencv_contrib" ]; then
  echo "Cloning OpenCV contrib 4.10.0..."
  git clone --branch 4.10.0 --depth 1 https://github.com/opencv/opencv_contrib.git
else
  echo "OpenCV contrib source already exists"
fi

# Build OpenCV
cd opencv
mkdir -p build
cd build

echo "Configuring OpenCV with contrib modules..."
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DOPENCV_EXTRA_MODULES_PATH=/workspace/opencv_contrib/modules \
  -DOPENCV_ENABLE_NONFREE=ON \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_DOCS=OFF \
  -DBUILD_PERF_TESTS=OFF \
  -DBUILD_TESTS=OFF \
  -DWITH_CUDA=OFF \
  -DWITH_QT=OFF \
  -DBUILD_opencv_python2=OFF \
  -DBUILD_opencv_python3=OFF

echo "Building OpenCV (this will take 1-2 hours on Raspberry Pi)..."
make -j4

echo "Installing OpenCV..."
make install

echo "Updating library cache..."
ldconfig

echo "=================================="
echo "OpenCV build complete!"
echo "=================================="
