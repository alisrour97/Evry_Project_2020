# Evry : TP Project

## Installation

You need Ubuntu 18.04 and ROS Melodic already installed. 

http://wiki.ros.org/melodic/Installation/Ubuntu

You need also git system : 

`sudo apt update && sudo apt install git` 

## Create your environment

First, if you do not have the catkin_ws folder, create it : 

`mkdir -p ~/catkin_ws/src`

Then go to this folder, and copy the project :

`cd ~/catkin_ws/src`

`git clone https://github.com/JohvanyROB/Evry_Project_2020.git`

Finally, compile the project : 

`cd ~/catkin_ws`

`catkin_make`

`source ~/catkin_ws/devel/setup.bash`

## Run the code

##### ON ONE TERMINAL : To run the Gazebo environment : 

`roslaunch evry_project_description simu_robot.launch group:=A nbr_robot:=3`

The arguments are : 

* group:=XXX : set the letter name of your group
* nbr_robot:=XXX : set the number of robot in your environment, maximum 3

##### ON AN ANOTHER TERMINAL : To run the robot's program : 

If you want to have the output of your code on <u>multiple</u> terminals : 

`roslaunch evry_project_strategy agent_terminal.launch group:=A nbr_robot:=3`

If you want to have the output of your code on a <u>single</u> terminal : 

`roslaunch evry_project_strategy agent.launch group:=A nbr_robot:=3`

## Edit the code

The robot's code is located at */evry_project_strategy/nodes/agent.py* 

You can edit directly the code itself and see the result.