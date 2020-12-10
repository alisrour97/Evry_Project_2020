#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import Range

from evry_project_plugins.srv import DistanceToFlag

class Robot:
    def __init__(self, group, robot_name, nb_flags):
        self.speed = 0.0
        self.angle = 0.0
        self.sonar = 0.0 #Sonar distance

        #ns : Name of the robot, like robot_A1, robot_A2 etc.
        #To be used for your subscriber and publisher with the robot itself
        self.group = group
        self.robot_name = robot_name
        self.ns = self.group + "/" + self.robot_name

        self.nb_flags = nb_flags    #Number of flags to discover in the environment

        '''Listener and publisher'''

        rospy.Subscriber(self.ns + "/sensor/sonar_front", Range, self.callbacksonar)
        self.cmd_vel_pub = rospy.Publisher(self.ns + "/cmd_vel", Twist, queue_size = 1)

        self.pub_velocity() #Run the publisher once

    def callbacksonar(self,data):
        self.sonar = data.range

    def get_sonar(self):
        return self.sonar

    def set_speed_angle(self,speed,angle):
        self.speed = speed
        self.angle = angle
        self.pub_velocity()

    def pub_velocity(self):
        self.speed = min(2, self.speed) # Maximum speed at 2 m/s

        cmd_vel = Twist()
        cmd_vel.linear.x = self.speed
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0

        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = self.angle

        self.cmd_vel_pub.publish(cmd_vel)

    def getDistanceToFlag(this):
        rospy.wait_for_service('/distanceToFlag')
        try:
            service = rospy.ServiceProxy('/distanceToFlag', DistanceToFlag)
            pose = Pose2D()
            pose.x = 0.0
            pose.y = 0.0
            pose.theta = 0.0
            result = service(pose)
            return (result.id_flag, result.distance)
        except rospy.ServiceException as e :
            print("Service call failed: %s"%e)

def run_demo():
    '''Main loop'''
    group = rospy.get_param("~group")
    robot_name = rospy.get_param("~robot_name")
    nb_flags = rospy.get_param("nb_flags")
    robot = Robot(group, robot_name, nb_flags)
    print("Robot : " + str(robot_name) +" from Group : " + str(group) + " is starting..")


    while not rospy.is_shutdown():
        #Write here your strategy..
        print("SONAR VALUE FOR "+str(robot_name)+" :")
        print(robot.get_sonar())

        print("Distance to flag : ")
        print(robot.getDistanceToFlag())

        velocity = 0
        angle = 0
        sonar = float(robot.get_sonar())


        #Finishing by publishing the desired speed. DO NOT TOUCH.
        robot.set_speed_angle(velocity,angle)
        rospy.sleep(0.5)




if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Controller", anonymous = True)


    run_demo()
