#!/bin/sh
set -e

sudo apt-get install -y ros-noetic-derived-object-msgs ros-noetic-carla-msgs
pip3 install transforms3d networkx  

current_directory=$(pwd)

rm -rf $current_directory/MPC_control_ex/External

cd /usr/include
sudo ln -sf eigen3/Eigen Eigen 
sudo ln -sf eigen3/unsupported unsupported
cd $current_directory/MPC_control_ex

# clone osqp
OSQP_repo="https://github.com/rise-lab-skku-racing/osqp.git"
local_path="External/osqp"
git clone "$OSQP_repo" "$local_path"

# clone osqp
OSQP_eigen_repo="https://github.com/rise-lab-skku-racing/osqp-eigen.git"
local_path="External/osqp-eigen"
git clone "$OSQP_eigen_repo" "$local_path"

cd External/osqp
mkdir -p build
mkdir -p lib
cd build
mkdir -p install
cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=$(realpath ../) ..
cmake --build .
sudo cmake --build . --target install
cd $current_directory/MPC_control_ex

cd External/osqp-eigen
mkdir -p build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$(realpath ../) -DCMAKE_PREFIX_PATH=$(realpath ../../osqp/lib/cmake/osqp) ..
make -j8
sudo make install -j8
