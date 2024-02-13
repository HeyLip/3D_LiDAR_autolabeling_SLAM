highlight "building DSP-SLAM ..."

if [ ! -d build ]; then
  mkdir build
fi
cd build
conda_python_bin=`which python3`
conda_env_dir="$(dirname "$(dirname "$conda_python_bin")")"

echo $conda_env_dir

cmake \
  -DCMAKE_BUILD_TYPE=Debug \
  -DOpenCV_DIR="$(pwd)/../Thirdparty/opencv/build" \
  -DEigen3_DIR="$(pwd)/../Thirdparty/eigen/install/share/eigen3/cmake" \
  -DPangolin_DIR="$(pwd)/../Thirdparty/Pangolin/build" \
  -DPYTHON_LIBRARIES="$conda_env_dir/lib/libpython3.7m.so" \
  -DPYTHON_INCLUDE_DIRS="$conda_env_dir/include/python3.7m" \
  -DPYTHON_EXECUTABLE="$conda_env_dir/bin/python3.7" \
  ..
make -j8
