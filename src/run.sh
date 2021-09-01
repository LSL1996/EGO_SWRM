

#roslaunch mavros px4.launch & sleep 1;
sudo chmod 777 /dev/ttyUSB0 & sleep 1;

rosrun serial_node myserial & sleep 1;

roslaunch realsense2_camera rs_camera.launch & sleep 10;

rosrun vins vins_node /home/hyy/ego_swarm/src/realflight_modules/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml & sleep 1;

#roslaunch px4ctrl run_ctrl.launch & sleep 1;


#roslaunch ego_planner run_in_exp.launch & sleep 1;

wait;
------------
