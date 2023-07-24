from controller import Camera, Supervisor, Robot, RangeFinder
import numpy as np
import math
import cv2
import time
from threading import Thread, Event

import rospy
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *
import tf

class Tiago (Supervisor):
    def findAndEnableDevices(self):
        # get the time step of the current world.
        self.timeStep = int(self.getBasicTimeStep())   
        print(self.timeStep)
        # Camera RGBD
        self.camera_color = self.getDevice("camera_color")
        self.camera_color.enable(33)
        self.camera_depth = self.getDevice("camera_depth")
        self.camera_depth.enable(33)

    def get_camera_image(self):
        color = self.camera_color.getImage()
        depth = self.camera_depth.getRangeImage(data_type="buffer")
        depth = np.ctypeslib.as_array(depth, (self.camera_depth.getWidth() * self.camera_depth.getHeight(),))

        ##### TESTING CAMERA #####

        color = cv2.cvtColor(np.frombuffer(color, np.uint8).reshape(self.camera_color.getHeight(), self.camera_color.getWidth(), 4), cv2.COLOR_RGBA2RGB )
        depth = np.ctypeslib.as_array(depth, (self.camera_depth.getWidth() * self.camera_depth.getHeight(),)).reshape(self.camera_depth.getHeight(), self.camera_depth.getWidth(), 1)

        cv2.imshow("color_image", color)
        cv2.imshow("depth_image", depth)
        cv2.waitKey(1)


    def __init__(self):
        Robot.__init__(self)
        self.findAndEnableDevices()

if __name__ == '__main__':
    tiago_robot = Tiago()

    while tiago_robot.step(tiago_robot.timeStep) != -1:  
        tiago_robot.get_camera_image()

