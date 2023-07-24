import sys
sys.path.append('/home/gerardo/software/webots/lib/controller/python')
from controller import Supervisor, Robot

import rospy
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *
import tf

import time
import ctypes




import numpy as np
import math
import argparse

class Person (Supervisor):
    def __init__(self, person_number):

        Robot.__init__(self)
        self.timeStep = int(self.getBasicTimeStep())   
        self.controller_functions = ctypes.CDLL('/home/gerardo/software/webots/lib/controller/libController.so', mode=ctypes.RTLD_GLOBAL)  
        self.bvh_functions = ctypes.CDLL('/home/gerardo/ros_projects/cohan_ws/src/CoHAN_Navigation/simulators/webots_ros/libraries/bvh_util/libbvh_util.so', mode=ctypes.RTLD_GLOBAL)  
        self.bvh_animation_functions = ctypes.CDLL('/home/gerardo/ros_projects/cohan_ws/src/CoHAN_Navigation/simulators/webots_ros/controllers/bvh_animation_mod/bvh_animation_mod.so')  
        self.bvh_animation_functions.load_motion_data(person_number)

        self.moving = False

        self.person_node = self.getFromDef("Human1")
        # self.speed_subscriber = rospy.Subscriber("/human1/cmd_vel", Twist, self.set_speed)
        self.speed_subscriber = rospy.Subscriber("/joy", Joy, self.set_speed)
        
    def set_speed(self, data):

        ########## In case the receibed speed is in the person frame ##########
        person_orientation = person.person_node.getOrientation()
        orientation = math.atan2(person_orientation[0], person_orientation[1]) - math.pi/2 
        rotation_matrix = np.array([[math.cos(orientation), -math.sin(orientation)],[math.sin(orientation), math.cos(orientation)]])
        lin_speed = np.array([data.axes[1] * 2, 0])
        if data.axes[1] > 0.1 or data.axes[1] < -0.1:
            self.moving = True
        else:
            self.moving = False
        converted_speed = np.matmul(rotation_matrix, lin_speed)
        print("SPEED SET", math.sqrt(converted_speed[0] ** 2 + converted_speed[1] ** 2))
        self.person_node.setVelocity([converted_speed[0], converted_speed[1], 0, 0, 0, data.axes[0] * math.pi/2])

        # self.person_node.setVelocity([data.linear.x, data.linear.y, 0, 0, 0, data.angular.z])

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--number', type=int)
    args = parser.parse_args()
    person_number = args.number

    if person_number:
        person = Person(person_number)     
    else:
        print("Person number required")
        exit()

   
    rospy.init_node("person_node")
    rospy.loginfo("person_node node has been started")

    rate = rospy.Rate(29.9) 
    person.bvh_animation_functions.motion_step()
    person.bvh_animation_functions.motion_step()
    while not rospy.is_shutdown(): 
        while person.step(person.timeStep) != -1:
            # person.bvh_animation_functions.cleanup()
            if person.moving:
                person.bvh_animation_functions.motion_step()
            try:
                rate.sleep()
            except KeyboardInterrupt:
                print("Shutting down")
