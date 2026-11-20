colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
sudo setcap cap_net_admin,cap_net_raw+eip build/rise_motion/rise_motion_main
source install/setup.bash
