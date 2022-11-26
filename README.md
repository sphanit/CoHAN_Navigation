
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
1. PR2 running CoHAN and stage GUI
	```
	roslaunch cohan_navigation stage_pr2_only.launch gui:=true map_name:=laas_adream
	```
2. PR2 and one human running CoHAN with GUI
	```
	roslaunch cohan_navigation stage_pr2_with_human_agent.launch gui:=true map_name:=laas_adream
	```
2. Running stage in fast mode
	```
	roslaunch cohan_navigation stage_pr2_only.launch fast_mode:=true map_name:=laas_adream
	```
# Running the examples (MORSE)
1. PR2 and one human running CoHAN and GUI
		Launch MORSE simulator scenario:
	```
	roslaunch morse_ros morse_pr2.launch map_name:=laas_adream with_humans:=true
	```
	In a separate terminal, launch the navigation:
	```
	roslaunch cohan_navigation morse_pr2_with_human_agent.launch map_name:=laas_adream localize:=amcl
	```
3. Running on a different map
	```
	roslaunch morse_ros morse_pr2.launch map_name:=maze with_humans:=true
	
	roslaunch cohan_navigation morse_pr2_only.launch map_name:=maze
