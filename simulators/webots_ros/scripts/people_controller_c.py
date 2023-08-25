import sys
import os
sys.path.append('/usr/local/webots/lib/controller/python')
from controller import Supervisor, Robot

import rospy
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *
from pedsim_msgs.msg import *
import tf
import time
import ctypes
import numpy as np
import math
import argparse

class Person (Supervisor):
    def __init__(self, person_number):
        parent = os.path.dirname(os.path.realpath(__file__))
        main_directory = os.path.abspath(os.path.join(parent, os.pardir))
        Robot.__init__(self)
        self.person_number = person_number
        self.timeStep = int(self.getBasicTimeStep())   
        self.controller_functions = ctypes.CDLL('/usr/local/webots/lib/controller/libController.so', mode=ctypes.RTLD_GLOBAL)  
        self.bvh_functions = ctypes.CDLL(main_directory + '/libraries/bvh_util/libbvh_util.so', mode=ctypes.RTLD_GLOBAL)  
        self.bvh_animation_functions = ctypes.CDLL(main_directory + '/controllers/bvh_animation_mod/bvh_animation_mod.so')  
        self.bvh_animation_functions.load_motion_data(person_number)

        self.moving = False

        self.person_node = self.getFromDef("Human"+str(person_number))
        self.traslation_field = self.person_node.getField('translation')
        self.rotation_field = self.person_node.getField('rotation')        

        self.pose_subscriber = rospy.Subscriber("/pedsim_visualizer/tracked_persons", TrackedPersons, self.set_pose)
        # self.speed_subscriber = rospy.Subscriber("/human1/cmd_vel", Twist, self.set_speed)
        if person_number == 1:
            self.speed_subscriber = rospy.Subscriber("joy", Joy, self.set_speed)
        
    def set_speed(self, data):
        print("SPEED")
        ########## In case the receibed speed is in the person frame ##########
        person_orientation = person.person_node.getOrientation()
        orientation = math.atan2(person_orientation[0], person_orientation[1]) - math.pi/2 
        rotation_matrix = np.array([[math.cos(orientation), -math.sin(orientation)],[math.sin(orientation), math.cos(orientation)]])
        lin_speed = np.array([data.axes[1] * 2, 0])
        # if data.axes[1] > 0.1 or data.axes[1] < -0.1:
        #     self.moving = True
        # else:
        #     self.moving = False
        converted_speed = np.matmul(rotation_matrix, lin_speed)
        self.person_node.setVelocity([converted_speed[0], converted_speed[1], 0, 0, 0, data.axes[0] * math.pi/2])

    def set_pose(self, data):
        person_position_data = data.tracks[person_number - 1].pose.pose.position   
        person_orientation_data = data.tracks[person_number - 1].pose.pose.orientation     
        self.traslation_field.setSFVec3f([person_position_data.x, person_position_data.y, 0])
        self.rotation_field.setSFRotation(self.quaternion_to_axis_angle([person_orientation_data.w, person_orientation_data.x, person_orientation_data.y, person_orientation_data.z]))
    
    def quaternion_to_axis_angle(self, quaternion):
        axis_angle = [0, 0, 0, 0]
        quaternion = np.array(quaternion)
        w, x, y, z = quaternion
        if w > 1:
            w, x, y, z = self.normalize_quaternion(quaternion)
        if w <= -1:
            axis_angle[3] = 2 * math.pi
        elif w < 1:
            axis_angle[3] = 2 * math.acos(w)  # Calcular el ángulo en radianes
        else:
            axis_angle[3] = 0

        if axis_angle[3] < 0.0001:
            return [0, 1, 0, 0]
        inv = 1 / math.sqrt(x * x + y * y + z * z)
        # print("quaternion_to_axis_angle", [x * inv, y * inv, z * inv, angle])
        axis_angle[0] = round(x * inv, 6)
        axis_angle[1] = round(y * inv, 6)
        axis_angle[2] = round(z * inv, 6)
        axis_angle[3] = round(axis_angle[3], 6)
        return axis_angle

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--number', type=int)
    args = parser.parse_args()
    person_number = args.number

    if person_number:
        os.environ["WEBOTS_CONTROLLER_URL"] = "ipc://1234/Human" + str(person_number)
        person = Person(person_number)     
    else:
        print("Person number required")
        exit()

    rospy.init_node("person_node_"+str(person_number))
    rospy.loginfo("person_node_"+str(person_number) +" node has been started")
    rate = rospy.Rate(30) 
    person.bvh_animation_functions.motion_step()
    person.bvh_animation_functions.motion_step()
    while not rospy.is_shutdown(): 
        while person.step(person.timeStep) != -1:
            # person.bvh_animation_functions.cleanup()
            # if person.moving:
            # if person_number == 1:
            person.bvh_animation_functions.motion_step()
            try:
                rate.sleep()
            except KeyboardInterrupt:
                print("Shutting down")
