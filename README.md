# Evry : TP Project

## Installation

You need Ubuntu 18.04 and ROS Melodic already installed. 

http://wiki.ros.org/melodic/Installation/Ubuntu

You need also git system : 

`sudo apt update && sudo apt install git` 

## Install rospkg for python3
`sudo pip3 install rospkg`

## Create your environment

In a terminal create catkin_ws folder, as follow: 

`mkdir -p ~/catkin_ws/src`

Then go to this folder, and copy the project :

`cd ~/catkin_ws/src`

`git clone https://github.com/JohvanyROB/Evry_Project_2020.git`

Finally, compile the project : 

`cd ~/catkin_ws`

`catkin_make`

`source ~/catkin_ws/devel/setup.bash`

`echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc` 

## Run the code

##### ON ONE TERMINAL : To run the Gazebo environment : 

`roslaunch evry_project_description simu_robot.launch group:=A nbr_robot:=3`

The arguments are : 

* group:=XXX : set the letter name of your group
* nbr_robot:=XXX : set the number of robot in your environment, maximum 3

##### ON ANOTHER TERMINAL : To run the robot's program : 

**1. If you want to have the output of your code on <u>multiple</u> terminals :** 

`roslaunch evry_project_strategy agent_terminal.launch group:=A nbr_robot:=3`

**2. If you want to have the output of your code on a <u>single</u> terminal :** 

`roslaunch evry_project_strategy agent.launch group:=A nbr_robot:=3`

## Edit the code

The robot's code is located at */evry_project_strategy/nodes/agent.py* 

You can edit directly the code itself and see the result.

![Gazebo's environment with robots](https://github.com/JohvanyROB/Evry_Project_2020/blob/main/Gazebo.PNG)

## Change the environment

By default, your robots operate in a simple environment with basic obstacles (lev1), but if you want to evaluate the robustness of your strategy, you can move to a more complex environment (lev2). 

You just need to modify the value of the argument **env** when launching the file **simu_robot.launch** as follow:

`roslaunch evry_project_description simu_robot.launch env:=lev2`
