#!/bin/bash
# Build a minimal OpenCV.js. Everything lands under the scratchpad.
set -e
S="$(cd "$(dirname "$0")" && pwd)"
cd "$S"

echo "### $(date +%H:%M:%S) emsdk"
if [ ! -d emsdk ]; then
  git clone --depth 1 https://github.com/emscripten-core/emsdk.git
fi
cd emsdk
./emsdk install 3.1.74 > emsdk-install.log 2>&1
./emsdk activate 3.1.74 >> emsdk-install.log 2>&1
source ./emsdk_env.sh > /dev/null 2>&1
cd "$S"
echo "### emcc: $(emcc --version | head -1)"

echo "### $(date +%H:%M:%S) opencv source"
if [ ! -d opencv ]; then
  git clone --depth 1 --branch 4.x https://github.com/opencv/opencv.git
fi

echo "### $(date +%H:%M:%S) building (this is the long part)"
cd opencv
python3 ./platforms/js/build_js.py "$S/out" \
  --build_wasm \
  --config "$S/minimal.config.py" \
  --cmake_option="-DBUILD_opencv_dnn=OFF" \
  --cmake_option="-DBUILD_opencv_photo=OFF" \
  --cmake_option="-DBUILD_opencv_objdetect=OFF" \
  --cmake_option="-DBUILD_opencv_ml=OFF" \
  --cmake_option="-DBUILD_opencv_gapi=OFF" \
  --cmake_option="-DBUILD_opencv_stitching=OFF" \
  --cmake_option="-DBUILD_opencv_video=ON" \
  --cmake_option="-DBUILD_opencv_calib3d=ON" \
  > "$S/build.log" 2>&1

echo "### $(date +%H:%M:%S) done"
ls -la "$S/out/bin/" 2>/dev/null
