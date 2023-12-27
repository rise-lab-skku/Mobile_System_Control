#!/bin/sh
set -e

current_directory=$(pwd)

# cd /usr/include
# sudo ln -sf eigen3/Eigen Eigen 
# sudo ln -sf eigen3/unsupported unsupported
# cd $current_directory
# echo "Current directory: $current_directory"

# clone osqp
# OSQP_repo="https://github.com/rise-lab-skku-racing/osqp.git"
# local_path="External/osqp"
# git clone "$OSQP_repo" "$local_path"

# clone osqp
OSQP_eigen_repo="https://github.com/rise-lab-skku-racing/osqp-eigen.git"
local_path="External/osqp-eigen"
git clone "$OSQP_eigen_repo" "$local_path"

# cd External/osqp
# mkdir -p build
# mkdir -p lib
# cd build
# cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=$(realpath ../) ..
# cmake --build .
# # sudo cmake -DCMAKE_INTALL_PATH=$(realpath ../lib) --build . --target install 
# sudo cmake --build . --target install
# cd $current_directory

cd External/osqp-eigen
mkdir -p build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$(realpath ../) -DOSQP_PATH=$(realpath ../../osqp/lib) ..
make -j8
sudo make install -j8
