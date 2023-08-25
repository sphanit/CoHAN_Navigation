#!/usr/bin/env python3

import sys
sys.path.append('/usr/local/webots/lib/controller/python')
from controller import Camera, Supervisor, Robot, RangeFinder, Lidar, Motor
#from controller import *
import numpy as np
import math
import cv2
import time
from threading import Thread, Event

import imagehash
from PIL import Image

import rospy
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *

import tf

class FourHolonomicWheelsRobot:
    def __init__(self, wheel_radius, robot_radius):
        self.wheel_radius = wheel_radius
        self.robot_radius = robot_radius

    def calculate_wheel_speeds(self, linear_velocity_x, linear_velocity_y, angular_velocity):
        v_FL = linear_velocity_x - linear_velocity_y - (angular_velocity * self.robot_radius)
        v_FR = linear_velocity_x + linear_velocity_y + (angular_velocity * self.robot_radius)
        v_RL = linear_velocity_x + linear_velocity_y - (angular_velocity * self.robot_radius)
        v_RR = linear_velocity_x - linear_velocity_y + (angular_velocity * self.robot_radius)
        return [v_FL, v_FR, v_RL, v_RR]

    def calculate_wheel_orientations(self, linear_velocity_x, linear_velocity_y, angular_velocity):
        orientation_FL = -linear_velocity_y - angular_velocity * (self.robot_radius + self.wheel_radius)
        orientation_FR = linear_velocity_y + angular_velocity * (self.robot_radius + self.wheel_radius)
        orientation_RL = linear_velocity_y - angular_velocity * (self.robot_radius + self.wheel_radius)
        orientation_RR = -linear_velocity_y + angular_velocity * (self.robot_radius + self.wheel_radius)
        return [orientation_FL, orientation_RL, orientation_FR, orientation_RR]

class Tiago (Supervisor):
    def findAndEnableDevices(self):
        # get the time step of the current world.
        self.timeStep = int(self.getBasicTimeStep())   
        self.robot_base = FourHolonomicWheelsRobot(wheel_radius=0.08, robot_radius=0.5)

        # Camera RGBD
        self.camera_color = self.getDevice("kinect color")
        self.camera_color.enable(33)
        self.camera_depth = self.getDevice("kinect range")
        self.camera_depth.enable(33)

        # Head joints
        # self.head_1 = self.getDevice("head_1_joint")
        # self.head_2 = self.getDevice("head_2_joint")
        # self.head_1.setPosition(0)
        # self.head_2.setPosition(0)

        # Torso joints
        # self.torso_lift_joint = self.getDevice("torso_lift_joint")
        # self.torso_lift_joint.setPosition(0)

        # Wheels
        front_left_wheels_names = ["fl_caster_l_wheel_joint", "fl_caster_r_wheel_joint"]
        front_right_wheels_names = ["fr_caster_l_wheel_joint", "fr_caster_r_wheel_joint"]
        back_left_wheels_names = ["bl_caster_l_wheel_joint", "bl_caster_r_wheel_joint"]
        back_right_wheels_names = ["br_caster_l_wheel_joint", "br_caster_r_wheel_joint"]

        self.front_left_wheels = []
        self.front_right_wheels = []
        self.back_left_wheels = []
        self.back_right_wheels = []

        for left_wheel in front_left_wheels_names:
            wheel = self.getDevice(left_wheel)
            wheel.setPosition(float('inf'))
            wheel.setVelocity(float(0))
            self.front_left_wheels.append(wheel)

        for right_wheel in front_right_wheels_names:
            wheel = self.getDevice(right_wheel)
            wheel.setPosition(float('inf'))
            wheel.setVelocity(float(0))
            self.front_right_wheels.append(wheel)

        for left_wheel in back_left_wheels_names:
            wheel = self.getDevice(left_wheel)
            wheel.setPosition(float('inf'))
            wheel.setVelocity(float(0))
            self.back_left_wheels.append(wheel)

        for right_wheel in back_right_wheels_names:
            wheel = self.getDevice(right_wheel)
            wheel.setPosition(float('inf'))
            wheel.setVelocity(float(0))
            self.back_right_wheels.append(wheel)

        # Wheel casters
        caster_wheels_names = ["bl_caster_rotation_joint", "br_caster_rotation_joint", "fl_caster_rotation_joint", "fr_caster_rotation_joint"]

        self.caster_wheels = []

        for caster_name in caster_wheels_names:
            caster = self.getDevice(caster_name)
            caster.setPosition(float('inf'))
            self.caster_wheels.append(caster)

        # LIDAR
        # self.lidar = self.getDevice("Hokuyo URG-04LX-UG01")
        # self.lidar.enable(100)
        # self.lidar.enablePointCloud()

    def get_camera_image(self, timestamp):
        color = self.camera_color.getImage()
        color_image = Image(data=color, height=self.camera_color.getHeight(), width=self.camera_color.getWidth())
        color_image.header.stamp = timestamp
        self.camera_color_publisher.publish(color_image)

        depth = self.camera_depth.getRangeImage(data_type="buffer")
        depth = np.ctypeslib.as_array(depth, (self.camera_depth.getWidth() * self.camera_depth.getHeight(),))
        depth_image = Image(data=depth.tobytes(), height=self.camera_depth.getHeight(), width=self.camera_depth.getWidth())
        depth_image.header.stamp = timestamp
        self.camera_depth_publisher.publish(depth_image)

        ##### TESTING CAMERA #####

        color = cv2.cvtColor(np.frombuffer(color, np.uint8).reshape(self.camera_color.getHeight(), self.camera_color.getWidth(), 4), cv2.COLOR_RGBA2RGB )
        depth = np.ctypeslib.as_array(depth, (self.camera_depth.getWidth() * self.camera_depth.getHeight(),)).reshape(self.camera_depth.getHeight(), self.camera_depth.getWidth(), 1)

        cv2.imshow("color_image", color)
        cv2.imshow("depth_image", depth)
        cv2.waitKey(1)
    
    def get_lidar_data(self, timestamp):
        sampling_period = self.lidar.getFrequency()
        lidar_min_range = self.lidar.getMinRange()
        lidar_max_range = self.lidar.getMaxRange()
        lidar_angle_range = self.lidar.getFov()
        lidar_range = self.lidar.getRangeImage()[::-1]
        lidar_intensities = []
        for i in range(len(lidar_range)):
            if lidar_range[i] == float('inf') or lidar_range[i] < 0.3:
                lidar_range[i] = lidar_max_range
                lidar_intensities.append(0)
            else:
                lidar_intensities.append(1)

        angle_increment = lidar_angle_range / len(lidar_range)
        laser_publish_data = LaserScan()
        laser_publish_data.header.stamp = timestamp
        laser_publish_data.header.frame_id = "base_laser_link"
        laser_publish_data.ranges = lidar_range
        laser_publish_data.intensities = lidar_intensities
        laser_publish_data.range_min = lidar_min_range
        laser_publish_data.range_max = lidar_max_range
        laser_publish_data.angle_min = -lidar_angle_range/2
        laser_publish_data.angle_max = -laser_publish_data.angle_min
        laser_publish_data.angle_increment = angle_increment
        laser_publish_data.scan_time = 1 / sampling_period
        self.lidar_publisher.publish(laser_publish_data)
        # yaw_to_euler = tf.transformations.quaternion_from_euler(0, 0, 0)       
        # self.tf_sender.sendTransform((0, 0, 0), yaw_to_euler, laser_publish_data.header.stamp, "base_laser_link", "base_link")

    def get_pose_data(self, timestamp):
        odometry_message = Odometry()
        odometry_message.header.stamp = timestamp
        odometry_message.header.frame_id = "odom"
        robot_pose_ground_truth = self.robot_node.getPosition()
        robot_speed = self.robot_node.getVelocity()
        robot_orientation = self.robot_node.getOrientation()
        odometry_message.pose.pose.position.x = robot_pose_ground_truth[0]
        odometry_message.pose.pose.position.y = robot_pose_ground_truth[1]
        odometry_message.pose.pose.position.z = robot_pose_ground_truth[2]
        
        robot_orientation_world_z = math.atan2(robot_orientation[3], robot_orientation[0])
        yaw_to_euler = tf.transformations.quaternion_from_euler(0, 0, robot_orientation_world_z)       
        odometry_message.pose.pose.orientation = Quaternion(x=yaw_to_euler[0], y=yaw_to_euler[1], z=yaw_to_euler[2], w=yaw_to_euler[3])
        rotation_matrix = np.array([[math.cos(robot_orientation_world_z), -math.sin(robot_orientation_world_z)],[math.sin(robot_orientation_world_z), math.cos(robot_orientation_world_z)]])
        lin_speed = np.array([robot_speed[0], robot_speed[1]])
        converted_speed = np.matmul(np.linalg.inv(rotation_matrix), lin_speed)
        odometry_message.twist.twist.linear.x = round(converted_speed[0], 3)
        odometry_message.twist.twist.linear.y = round(converted_speed[1], 3)
        odometry_message.twist.twist.angular.z = round(robot_speed[5], 3)
        # self.tf_sender.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0), timestamp, "base_footprint", "odom")
        self.tf_sender.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0), timestamp, "base_link", "base_footprint")
        self.tf_sender.sendTransform((robot_pose_ground_truth[0], robot_pose_ground_truth[1], 0), yaw_to_euler, timestamp, "base_footprint", "odom")
        self.odom_publisher.publish(odometry_message)

        self.pose_ground_truth_publisher.publish(odometry_message)
        # self.tf_sender.sendTransform((robot_pose_ground_truth[0], robot_pose_ground_truth[1], robot_pose_ground_truth[2]), tf.transformations.quaternion_from_euler(0, 0, robot_orientation_world_z), timestamp, "map", "xtion_link")
        
    ################## SPEED COMMANDS ##################

    def set_speed(self, data):
        lin_speed = data.linear.x
        ang_speed = data.angular.z
        left, right = self.speed_commands_to_differential_drive(lin_speed, ang_speed)
        left = np.clip(left, -10.1523, 10.1523)
        right = np.clip(right, -10.1523, 10.1523)
        for wheel in self.left_wheels:
            wheel.setVelocity(left)
        for wheel in self.right_wheels:
            wheel.setVelocity(right)

    def set_speed_joy(self, data):
        lin_speed = data.axes[4]
        ang_speed = data.axes[3]
        wheel_orientations = self.robot_base.calculate_wheel_orientations(lin_speed, 0, ang_speed)
        print("wheel_orientations", wheel_orientations)
        for i, caster in enumerate(self.caster_wheels):
            caster.setPosition(wheel_orientations[i])
        wheel_speeds = self.robot_base.calculate_wheel_speeds(lin_speed * 10, 0, ang_speed * 10)

        print("wheel_speeds", wheel_speeds)
        for wheel in self.front_left_wheels:
            wheel.setVelocity(wheel_speeds[0])
        for wheel in self.front_right_wheels:
            wheel.setVelocity(wheel_speeds[1])
        for wheel in self.back_left_wheels:
            wheel.setVelocity(wheel_speeds[2])
        for wheel in self.back_right_wheels:
            wheel.setVelocity(wheel_speeds[3])

    
    def publish_data(self):
        act_timestamp = rospy.Time.now()
        self.get_camera_image(act_timestamp)
        self.get_pose_data(act_timestamp)        
        # self.get_lidar_data(act_timestamp)

    def cmd_vel_thread(self, event: Event):
        while not event.is_set():
            self.speed_subscriber = rospy.Subscriber("/joy", Joy, self.set_speed_joy)
            # self.speed_subscriber = rospy.Subscriber("/cmd_vel", Twist, self.set_speed)
            rospy.spin()
        
    def __init__(self):
        Robot.__init__(self)
        self.findAndEnableDevices()
        self.odom_publisher = rospy.Publisher("/odom", Odometry, queue_size=5)
        # self.camera_odom_publisher = rospy.Publisher("/camera_odom", Odometry, queue_size=5)
        self.pose_ground_truth_publisher = rospy.Publisher("/base_pose_ground_truth", Odometry, queue_size=5)
        # self.odom_publisher = rospy.Publisher("/base_pose_ground_truth", Odometry, queue_size=10)
        self.camera_color_publisher = rospy.Publisher("/xtion/rgb/image_raw", Image, queue_size=5)
        self.camera_depth_publisher = rospy.Publisher("/xtion/depth/image_raw", Image, queue_size=5)
        self.lidar_publisher = rospy.Publisher("/base_scan", LaserScan, queue_size=5)
        self.tf_sender = tf.TransformBroadcaster()

        self.robot_node = self.getFromDef("PR2")
        
        # self.camera_pose_respect_robot = list(map(lambda x,y: x-y ,self.camera_node.getPosition(),self.robot_node.getPosition()))
        self.thread_period = 20
        self.event = Event()
        self.vel_thread = Thread(target=self.cmd_vel_thread, args=[self.event],
                                      name="cmd_vel_thread", daemon=True)
        self.vel_thread.start()

if __name__ == '__main__':
    rospy.init_node("tiago_robot")
    rospy.loginfo("tiago_robot node has been started")
    tiago_robot = Tiago()
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():   
        while tiago_robot.step(tiago_robot.timeStep) != -1:  
            tiago_robot.publish_data()
            try:
                rate.sleep()
            except KeyboardInterrupt:
                print("Shutting down")


