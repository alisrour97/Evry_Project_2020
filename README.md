# Evry : TP Project

## Installation

You need Ubuntu 18.04 and ROS Melodic already installed. 

http://wiki.ros.org/melodic/Installation/Ubuntu

You need also git system : 

`sudo apt update && sudo apt install git` 

## Install rospkg for python3
```bash
sudo apt install python3-pip
sudo pip3 install rospkg
```

## Create your environment

In a terminal create catkin_ws folder, as follow: 

`mkdir -p ~/catkin_ws/src`

Then go to this folder, and copy the project :

```bash
cd ~/catkin_ws/src && git clone https://github.com/JohvanyROB/Evry_Project_2020.git
```

Finally, compile the project : 

```bash
cd ~/catkin_ws && catkin_make
source ~/catkin_ws/devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

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

## The Algorithm Approach

Mobile robots currently are used in various applications ranging from military, industrial and humanitarian applications.Several and various algorithms have been developed to help guide mobile robots to navigate in unknown environment like A star, Dijkstra, Voroni graphs, RRT and more.

In this project we need find a strategy to navigate to all flags in gazebo environment taking into consideration the noisy environment and the unknown obstacles.

This algorithm depends on continuously sensing the distance between the robot and the flag using some heuristic approach upon depending on small forward and backward movements, after this movements the robot can easily determine whether it should move in forward, backward, to the right or to the left and these movements are repeated until the robot reaches the desired flag. 

## The Flowchart of The Algorithm

![ROS2](https://user-images.githubusercontent.com/46069427/106035955-fe41f200-60d4-11eb-87cb-0afd7d5b46ee.png)


## Description

To facilitate the understanding of this algorithm, we have developed the above flow chart which well describes the algorithm in details. This flow chart service as the base point where we converted it into a python code.
Basically, this algorithm  depends on small displacement of forward and backward movements. In this displacement, the robot senses the ID of the flag and the distance to it. Based on these distances and ID’s found, we can properly guide the robot towards the flag in the four directions. This approach depends on continuously moving in small steps toward the goal to minimize the flag distance until we reach the flag/goal. We always keep in mind that the first priority of the robot is to avoid obstacles, that is why we always take the measurement of the sonar sensor and if the distance is less than a certain number, we activate the avoid obstacle function.

## Robots Cooperation

The mission coordination part is set such that when a robot finds a flag, it publishes it via a topic where all the robots can update their list of visited flags. So instead of one robot searching for all the flags, the three robots will cooperate with each other to fill the list of visited flags.

## Testing and Validation

The algorithm was very successful and was well adapted to the problem that we have. It can reach very high accuracies of less than 1-meter error in the flags position. A link to the video of first algorithm deployed for the three robots can be found on YouTube: https://youtu.be/zDzIvGVdQqg 



