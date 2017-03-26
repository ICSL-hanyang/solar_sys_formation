# solar_sys_formation

본인의 Firmware 디렉토리로 가서

source Tools/setup_gazebo.bash $(pwd) $(pwd)/build_posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch solar_sys_formation solar_sys_formation_sim.launch



roslaunch solar_sys_formation solar_sys_ros.launch


alias m0='rosparam set /earth_node/mode 0 && rosparam set /moon_node/mode 0 && rosparam set /satellite_node/mode 0'
alias m1='rosparam set /earth_node/mode 1 && rosparam set /moon_node/mode 1 && rosparam set /satellite_node/mode 1'
alias m2='rosparam set /earth_node/mode 2 && rosparam set /moon_node/mode 2 && rosparam set /satellite_node/mode 2'
alias m2='rosparam set /earth_node/mode 2 && rosparam set /moon_node/mode 2 && rosparam set /satellite_node/mode 2'
alias m3='rosparam set /earth_node/mode 3 && rosparam set /moon_node/mode 3 && rosparam set /satellite_node/mode 3'
