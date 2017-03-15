echo "Building ROS nodes"

cd Examples/ROS/ORB_SLAM2
mkdir build_debug	
cd build_debug
cmake .. -DROS_BUILD_TYPE=Debug
make -j
