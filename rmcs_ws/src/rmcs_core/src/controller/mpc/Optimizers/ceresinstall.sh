cd ~ && git clone https://ceres-solver.googlesource.com/ceres-solver  
cd ceres-solver 
mkdir build && cd build && cmake ..
make -j4
sudo make install

sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen