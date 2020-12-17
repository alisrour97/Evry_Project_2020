#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose2D
# TD5 : We add the odometry message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

import math

from evry_project_plugins.srv import DistanceToFlag

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z # in radians

class Robot:
    def __init__(self, group, robot_name, nb_flags):
        self.speed = 0.0
        self.angle = 0.0
        self.sonar = 0.0 #Sonar distance

        # TD5 : We add the pose of the robot as global parameters
        self.x = 0.0
        self.y = 0.0
        # Quaternion
        self.yaw = 0.0 # Yaw angle rotation
        #ns : Name of the robot, like robot_A1, robot_A2 etc.
        #To be used for your subscriber and publisher with the robot itself
        self.group = group
        self.robot_name = robot_name
        self.ns = self.group + "/" + self.robot_name

        self.nb_flags = nb_flags    #Number of flags to discover in the environment

        '''Listener and publisher'''

        rospy.Subscriber(self.ns + "/sensor/sonar_front", Range, self.callbacksonar)
        self.cmd_vel_pub = rospy.Publisher(self.ns + "/cmd_vel", Twist, queue_size = 1)
        # TD5 : We listen the pose of the robot
        rospy.Subscriber(self.ns + "/odom", Odometry, self.callbackpose)

        self.pub_velocity() #Run the publisher once

    def callbackpose(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.orientation.y
        qz = data.pose.pose.orientation.z
        qw = data.pose.pose.orientation.w

        _, _, self.yaw = euler_from_quaternion(qx, qy, qz, qw)
        print(self.yaw)

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

    def getDistanceToFlag(self):
        rospy.wait_for_service('/distanceToFlag')
        try:
            service = rospy.ServiceProxy('/distanceToFlag', DistanceToFlag)
            pose = Pose2D()
            # TD5 : We update the pose into the service
            pose.x = self.x
            pose.y = self.y
            pose.theta = 0.0
            result = service(pose)
            return result.distance
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
