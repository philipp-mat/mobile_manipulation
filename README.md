# Launch mobile panda controller

# For twist controller simulation

# 1. Terminal
roslaunch mobile_panda_simulation mobile_panda_sim.launch

# 2. Terminal
roslaunch mobile_panda_controller mobile_panda_controller_sim.launch controller:=mobile_panda_twist_controller

# 3. Terminal
rqt (robot stering to change position)

# For path controller simulation

# 1. Terminal
roslaunch mobile_panda_simulation mobile_panda_sim.launch

# 2. Terminal
roslaunch mobile_panda_controller mobile_panda_controller_sim.launch controller:=mobile_panda_path_controller

# Rebuild workspace
cd /home/philipp/mobile_manipulation/mobile_manipulation_ws
catkin build -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/home/philipp/mobile_manipulation/libfranka/build