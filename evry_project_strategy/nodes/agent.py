#!/usr/bin/env python2

import time
import numpy as np
import math
import rospy
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

from evry_project_plugins.srv import DistanceToFlag


class Robot:
    def __init__(self, group, robot_name, nb_flags):
        self.speed = 0.0
        self.angle = 0.0
        self.sonar = 0.0 #Sonar distance
        self.positionx = 0.0
        self.positiony = 0.0
        self.theta = 0.0
        self.IDFlag = 0.0
        self.DFlag = 100 #initialize high value
        self.VFlag = [] # Visited flag list
        self.IDPOINTFLAG = 0


        #ns : Name of the robot, like robot_A1, robot_A2 etc.
        #To be used for your subscriber and publisher with the robot itself
        self.group = group
        self.robot_name = robot_name
        self.ns = self.group + "/" + self.robot_name

        ns1 = self.group + "/" + "robot_1"
        ns2 = self.group + "/" + "robot_2"
        ns3 = self.group + "/" + "robot_3"


        self.nb_flags = nb_flags    #Number of flags to discover in the environment

        '''Listener and publisher'''

        rospy.Subscriber(self.ns + "/sensor/sonar_front", Range, self.callbacksonar)
        rospy.Subscriber(self.ns + "/odom", Odometry, self.callbackodom)
        # we will publish all the ids of the flags on the topic of data type Point which can be found in
        # Geometry messages
        rospy.Subscriber(ns1 + "/point", Point, self.callbackPoint)
        rospy.Subscriber(ns2 + "/point", Point, self.callbackPoint)
        rospy.Subscriber(ns3 + "/point", Point, self.callbackPoint)



        self.cmd_vel_pub = rospy.Publisher(self.ns + "/cmd_vel", Twist, queue_size = 1)
        self.Visited_Point_flag_ID = rospy.Publisher(self.ns + "/point", Point, queue_size = 3)

        self.pub_velocity() #Run the publisher once

    def callbacksonar(self,data):
        self.sonar = data.range

    def callbackPoint(self, dataID):
        self.VFlag.append(int(dataID.x))

    def get_ID_discovered_by_another_robot(self):
        return self.IDPOINTFLAG


    def callbackodom (self, message):

        self.positionx = message.pose.pose.position.x
        self.positiony = message.pose.pose.position.y
        self.theta = math.atan2(self.positiony, self.positionx)

    def get_position(self):
        return self.positionx, self.positiony

    def get_sonar(self):
        return self.sonar

    def move_forward(self):
        self.speed = 4
        self.angle = 0
        return self.speed, self.angle

    def move_backward(self):
        self.speed = -4
        self.angle = 0
        return self.speed, self.angle

    # a celebration move when we reach the flag
    def move_celebration(self):
        self.speed = 3
        self.angle = 1.57
        return self.speed, self.angle




    #function for object avoidance

    def object_avoidance(self,sonar):
        self.speed = 1.1
        self.angle = 1.8
        return self.speed, self.angle


    def set_speed_angle(self,speed,angle):
        self.speed = speed
        self.angle = angle
        self.pub_velocity()

    def pub_velocity(self):
        self.speed = min(5, self.speed) # Maximum speed at 2 m/s

        cmd_vel = Twist()
        cmd_vel.linear.x = self.speed
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0

        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = self.angle

        self.cmd_vel_pub.publish(cmd_vel)

    def set_ID_Visited(self, ID):
        self.IDPOINTFLAG = ID
        print("I have added this flag to the list of discovered flags for all robots by publishing it")
        self.pub_IDFlag()

    def pub_IDFlag(self):

        ID = Point()
        ID.x = self.IDPOINTFLAG
        ID.y = 0
        ID.z = 0
        self.Visited_Point_flag_ID.publish(ID)


    def getDistanceToFlag(this):
        rospy.wait_for_service('/distanceToFlag')
        try:
            service = rospy.ServiceProxy('/distanceToFlag', DistanceToFlag)
            pose = Pose2D()
            pose.x = this.positionx
            pose.y = this.positiony
            pose.theta = this.theta
            result = service(pose)
            #print("This is to print results")
            #print(result)
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
    initial = 1


    while not rospy.is_shutdown():


        # My Method###############################################
        # so if all flags where discovered, we print the end of the program and the visited flags in order of visit
        if len(robot.VFlag) == 8:
            print("End of the program")
            print("The list of discovered flags in order of discovery is:  ", robot.VFlag)
            print("ALL ROBOTS STOP, AMAZING WORK")
            print("THE END OF TESLA ALGORITHM")
            print("Done by Ali SROUR")
            robot.set_speed_angle(0, 0)
            # sleep for a very long time which defines the end of the program or we can shutdown
            rospy.sleep(10000000)

        else:

            if robot.DFlag < 2:
                # this is to make sure before publishing the visited flag, that it is not found in Visited flag list
                b = 0
                for i in robot.VFlag:
                    if robot.IDFlag == i:
                        b = 1
                # if it is not found then we publish it
                if b == 0:
                    print("we have reached the flag of ID "+ str(robot.IDFlag) +"  Horrayyyyyyyyyyyyyyyyy")
                    velocity, angle = robot.move_celebration()
                    robot.set_speed_angle(velocity, angle)

                    print(" Celebration with a loop, we earned a new flag, Hoooooorayyyyyyyyyyyyyyyyyyy! Amazing accomplishment ")

                    robot.set_ID_Visited(robot.IDFlag)
                    rospy.sleep(2.5)
                #robot.VFlag.append(robot.IDFlag)
                print("This is the list of discovered flags", robot.VFlag)
                l = 8 - len(robot.VFlag)
                print("Remaining number of flags to be discovered is", l)
                breaker = 1
                print("Time to search for another flag ID")

                # here we do a while loop such that to search for another flag away from one that we have just discovered
                while breaker == 1 :


                    sonar = float(robot.get_sonar())
                    if sonar < 4:
                        velocity, angle = robot.object_avoidance(sonar)
                        robot.set_speed_angle(velocity, angle)
                        rospy.sleep(1)
                    else:
                        robot.set_speed_angle(3, 0)
                        rospy.sleep(0.2)



                    Id, d = robot.getDistanceToFlag()
                    robot.IDFlag = Id
                    robot.DFlag = d
                    for i in robot.VFlag :

                        if robot.IDFlag == i:
                            breaker = 1
                            break
                        else:

                            breaker = 0

                    if len(robot.VFlag) == 8:
                        breaker = 0

            else:
                counter2 = 0
                counter = 0
                Id, d = robot.getDistanceToFlag()
                robot.IDFlag = Id
                robot.DFlag = d + 0.57 #estimated average error

                while robot.DFlag > 2 :

                    bb = 0
                    for i in robot.VFlag:
                        if robot.IDFlag == i:
                            bb = 1

                    if bb == 1:
                        break



                    if len(robot.VFlag) == 8:
                        break

                    sonar = float(robot.get_sonar())

                    if sonar < 4.5:
                        velocity, angle = robot.object_avoidance(sonar)
                        robot.set_speed_angle(velocity, angle)
                        rospy.sleep(1)

                    else:

                        # The initialisation is run only one time.
                        # we then set value initial to zero and will never become 1 again
                        # First Initialisation only to seperate the robots
                        if initial == 1 and robot.robot_name == 'robot_1':
                            robot.set_speed_angle(4, 0.03)
                            print("initialization robot 1 , once")
                            rospy.sleep(11)
                            initial = 0
                        elif initial == 1 and robot.robot_name == 'robot_2':
                            robot.set_speed_angle(0, 0.01)
                            print("initialization robot 2, once")
                            rospy.sleep(0)
                            initial = 0
                        elif initial == 1 and robot.robot_name == 'robot_3':
                            robot.set_speed_angle(-3, -0.075)
                            print("initialization robot 3, once")
                            rospy.sleep(10.5)
                            initial = 0

                        # moving forward
                        velocity, angle = robot.move_forward()
                        robot.set_speed_angle(velocity, angle)
                        rospy.sleep(0.7)

                        #collect forward distance and id
                        Id_f, D1 = robot.getDistanceToFlag()

                        print("This is the value of D1", D1)
                        print("This is the value of robot in middle position D ", robot.DFlag)
                        # moving backward
                        velocity, angle = robot.move_backward()
                        robot.set_speed_angle(velocity, angle)
                        rospy.sleep(1.4)

                        # collect backward distance and ID
                        Id_b, D2 = robot.getDistanceToFlag()
                        print("This is the value of D2", D2)
                        # moving back to the middle
                        velocity, angle = robot.move_forward()
                        robot.set_speed_angle(velocity, angle)
                        rospy.sleep(0.7)



                        if robot.IDFlag == Id_f and robot.DFlag > D1:
                            #Move forward
                            velocity, angle = robot.move_forward()
                            robot.set_speed_angle(velocity, angle)
                            rospy.sleep(0.5)
                            Id, robot.DFlag = robot.getDistanceToFlag()
                        else:
                            if robot.IDFlag == Id_b and robot.DFlag > D2:
                                # Move backward
                                velocity, angle = robot.move_backward()
                                robot.set_speed_angle(velocity, angle)
                                rospy.sleep(0.5)
                                Id, robot.DFlag = robot.getDistanceToFlag()
                                # because we dont have a rear sonar sensor, if we are stuck in backward position
                                # then we change a little bit our position
                                # counter2 is initialized to zero and will be reset after 9 iteration
                                # This means if we go too much in this loop, the robot will just rotate 90 degrees
                                counter2 = counter2 + 1
                                if counter2 == 9:
                                    counter2 = 0
                                    robot.set_speed_angle(0.5, 1.57)
                                    rospy.sleep(0.65)


                            else:
                                if robot.IDFlag == Id_f or robot.IDFlag == Id_b:
                                    #rotate 90 degrees to search on lateral direction
                                    robot.set_speed_angle(0, 1.57)
                                    rospy.sleep(1.0)
                                    counter = counter + 1
                                    #means if we entered too much for this loop for the same flag
                                    # we then have entered a dead zone of repeating steps
                                    if counter == 8:
                                        robot.set_speed_angle(2, 0.2)
                                        rospy.sleep(0.9)
                                        counter = 0


                                else:
                                    #random movement
                                    print ("I have lost my flag and I will search for another flag")
                                    robot.set_speed_angle(4, 0.2)
                                    rospy.sleep(2)
                                    robot.IDFlag, robot.DFlag = robot.getDistanceToFlag()





if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Controller", anonymous = True)


    run_demo()

