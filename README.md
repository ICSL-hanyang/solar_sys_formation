# solar_sys_formation

본인의 Firmware 디렉토리로 가서
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build_posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch solar_sys_formation solar_sys_formation_sim.launch



roslaunch solar_sys_formation solar_sys_ros.launch

