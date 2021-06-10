
# CoHAN Navigation

This package provides the configuration files and examples for the [CoHAN Planner](https://github.com/sphanit/CoHAN_Planner).

It uses slightly modified versions of [stage_ros](https://github.com/ros-simulation/stage_ros) and [MORSE](https://github.com/morse-simulator/morse) simulators for simulating the robot and the humans. The modified version of ```stage_ros``` is already included and the installation guide for ```MORSE``` can be found [here](https://github.com/sphanit/morse/blob/cohan_melodic/installation.md).

# Installation
1. Download and build the package
	```
	cd cohan_ws/src
	git clone https://github.com/sphanit/CoHAN_Navigation.git -b multi
	cd ..
	rosdep install --from-paths src --ignore-src --rosdistro=melodic -y
	catkin build
	```
	(install any other depencies if needed)
	
2. Install MORSE seperately before running its examples.
# Running the examples (Stage)
Needs to be updated
