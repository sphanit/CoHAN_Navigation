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
import os
import argparse

import imagehash
from PIL import Image

from transforms3d.euler import euler2mat, mat2euler
from transforms3d.axangles import axangle2mat, mat2axangle

import rospy
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *
from trajectory_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError

import tf

class FourHolonomicWheelsRobot:
    def __init__(self):
        self.diameter = 0.08  # caster diameter
        self.offset_x = 0.15  # caster offset
        self.offset_y = 0.15
    def calculate_wheel_parameters(self, linear_velocity_x, linear_velocity_y, angular_velocity):
        fr_v_x = linear_velocity_x - angular_velocity * (-self.offset_y)
        fr_v_y = linear_velocity_y + angular_velocity * (self.offset_x)
        fl_v_x = linear_velocity_x - angular_velocity * (self.offset_y)
        fl_v_y = linear_velocity_y + angular_velocity * (self.offset_x)
        br_v_x = linear_velocity_x - angular_velocity * (-self.offset_y)
        br_v_y = linear_velocity_y + angular_velocity * (-self.offset_x)
        bl_v_x = linear_velocity_x - angular_velocity * (self.offset_y)
        bl_v_y = linear_velocity_y + angular_velocity * (-self.offset_x)
        # v[m/s] = r[rad/s] * 0.1[m]  ## 0.1 = diameter
        fr_v = math.hypot(fr_v_x, fr_v_y)
        fl_v = math.hypot(fl_v_x, fl_v_y)
        br_v = math.hypot(br_v_x, br_v_y)
        bl_v = math.hypot(bl_v_x, bl_v_y)
        fr_a = math.atan2(fr_v_y, fr_v_x)
        fl_a = math.atan2(fl_v_y, fl_v_x)
        br_a = math.atan2(br_v_y, br_v_x)
        bl_a = math.atan2(bl_v_y, bl_v_x)

        # fix for -pi/2 - pi/2
        # fr_v = -fr_v if math.fabs(fr_a) > math.pi / 2 else fr_v
        # fr_a = fr_a - math.pi if fr_a > math.pi / 2 else fr_a
        # fr_a = fr_a + math.pi if fr_a < -math.pi / 2 else fr_a
        # fl_v = -fl_v if math.fabs(fl_a) > math.pi / 2 else fl_v
        # fl_a = fl_a - math.pi if fl_a > math.pi / 2 else fl_a
        # fl_a = fl_a + math.pi if fl_a < -math.pi / 2 else fl_a
        # br_v = -br_v if math.fabs(br_a) > math.pi / 2 else br_v
        # br_a = br_a - math.pi if br_a > math.pi / 2 else br_a
        # br_a = br_a + math.pi if br_a < -math.pi / 2 else br_a
        # bl_v = -bl_v if math.fabs(bl_a) > math.pi / 2 else bl_v
        # bl_a = bl_a - math.pi if bl_a > math.pi / 2 else bl_a
        # bl_a = bl_a + math.pi if bl_a < -math.pi / 2 else bl_a

        return [-1*fl_a, -1*fr_a, -1*bl_a, -1*br_a, np.clip(fl_v/(self.diameter / 2.0), -14, 14), np.clip(fr_v/(self.diameter / 2.0), -14, 14), np.clip(bl_v/(self.diameter / 2.0), -14, 14), np.clip(br_v/(self.diameter / 2.0), -14, 14)]

class PR2 (Supervisor):
    def findAndEnableDevices(self):
        # get the time step of the current world.
        self.robot_caster_velocity = 0
        self.timeStep = int(self.getBasicTimeStep())   
        self.robot_base = FourHolonomicWheelsRobot()

        # Camera RGBD
        self.camera_color = self.getDevice("camera_color")
        self.camera_color.enable(33)
        self.camera_depth = self.getDevice("camera_depth")
        self.camera_depth.enable(33)
        self.camera_node = self.getFromDef("rgb")

        # Head joints
        self.head_joint = self.getFromDef("head_pan_link")

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
            wheel.setPosition(float('+inf'))
            wheel.setVelocity(float(0.0))
            self.front_left_wheels.append(wheel)

        for right_wheel in front_right_wheels_names:
            wheel = self.getDevice(right_wheel)
            wheel.setPosition(float('+inf'))
            wheel.setVelocity(float(0.0))
            self.front_right_wheels.append(wheel)

        for left_wheel in back_left_wheels_names:
            wheel = self.getDevice(left_wheel)
            wheel.setPosition(float('+inf'))
            wheel.setVelocity(float(0.0))
            self.back_left_wheels.append(wheel)

        for right_wheel in back_right_wheels_names:
            wheel = self.getDevice(right_wheel)
            wheel.setPosition(float('+inf'))
            wheel.setVelocity(float(0.0))
            self.back_right_wheels.append(wheel)

        # Wheel casters
        caster_wheels_names = ["bl_caster_rotation_joint", "br_caster_rotation_joint", "fl_caster_rotation_joint", "fr_caster_rotation_joint"]

        self.caster_wheels = []

        for caster_name in caster_wheels_names:
            caster = self.getDevice(caster_name)
            caster.setPosition(float('+inf'))
            self.robot_caster_velocity = caster.getVelocity()
            caster.setVelocity(float(0.0))
            self.caster_wheels.append(caster)

        # LIDAR
        self.lidar = self.getDevice("base_laser")
        self.lidar.enable(100)
        self.lidar.enablePointCloud()

    def get_camera_image(self, timestamp):
        color = self.camera_color.getImage()
        color_image = cv2.cvtColor(cv2.cvtColor(np.frombuffer(color, np.uint8).reshape(self.camera_color.getHeight(), self.camera_color.getWidth(), 4), cv2.COLOR_RGBA2RGB ), cv2.COLOR_BGR2RGB)
        color_image = self.cv_bridge.cv2_to_imgmsg(color_image)
        color_image.header.stamp = timestamp
        self.camera_color_publisher.publish(color_image)

        depth = self.camera_depth.getRangeImage(data_type="buffer")
        depth_cv = np.ctypeslib.as_array(depth, (self.camera_depth.getWidth() * self.camera_depth.getHeight(),)).reshape(self.camera_depth.getHeight(), self.camera_depth.getWidth(), 1)
        depth_cv = np.multiply(depth_cv, 1000).astype(np.uint16)
        depth_cv = self.cv_bridge.cv2_to_imgmsg(depth_cv)
        depth_cv.header.stamp = timestamp
        self.camera_depth_publisher.publish(depth_cv)

        ##### TESTING CAMERA #####

        color = cv2.cvtColor(np.frombuffer(color, np.uint8).reshape(self.camera_color.getHeight(), self.camera_color.getWidth(), 4), cv2.COLOR_RGBA2RGB )
        depth = np.ctypeslib.as_array(depth, (self.camera_depth.getWidth() * self.camera_depth.getHeight(),)).reshape(self.camera_depth.getHeight(), self.camera_depth.getWidth(), 1)

        # cv2.imshow("color_image", color)
        # cv2.imshow("depth_image", depth)
        # cv2.waitKey(1)
    
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
        
    def update_head_rotation(self, timestamp):
        act_rotation = self.head_joint.getField('rotation').getSFRotation()
        # (trans, rot) = self.tf_listener.lookupTransform("/base_link", "/head_tilt_link", rospy.Time(0))
        # self.tf_sender.sendTransform((trans[0], trans[1], trans[2]), tf.transformations.quaternion_from_euler(0, 0, act_rotation[3] * act_rotation[2]), timestamp, "head_tilt_link", "base_link")
        (trans, rot) = self.tf_listener.lookupTransform("/torso_lift_link", "/head_pan_link", rospy.Time(0))
        self.tf_sender.sendTransform((trans[0], trans[1], trans[2]), tf.transformations.quaternion_from_euler(0, 0, act_rotation[3] * act_rotation[2]), timestamp, "head_pan_link", "torso_lift_link")
    ################## SPEED COMMANDS ##################

    def set_speed(self, data):
        lin_x_speed = data.linear.x
        lin_y_speed = data.linear.y
        ang_speed = data.angular.z
        print("SPEEDS", lin_x_speed, -lin_y_speed, ang_speed)
        wheel_parameters = self.robot_base.calculate_wheel_parameters(lin_x_speed, -lin_y_speed, ang_speed)
        print("WHEEL PARAMETERS", wheel_parameters)
        for i, caster in enumerate(self.caster_wheels):
            caster.setPosition(wheel_parameters[i])
            caster.setVelocity(5.0)
        for wheel in self.front_left_wheels:
            wheel.setVelocity(wheel_parameters[4])
            
        for wheel in self.front_right_wheels:
            wheel.setVelocity(wheel_parameters[5])
        for wheel in self.back_left_wheels:
            wheel.setVelocity(wheel_parameters[6])
        for wheel in self.back_right_wheels:
            wheel.setVelocity(wheel_parameters[7])

    def set_speed_joy(self, data):
        lin_x_speed = data.axes[1]
        lin_y_speed = data.axes[0]
        ang_speed = data.axes[3]
        wheel_parameters = self.robot_base.calculate_wheel_parameters(lin_x_speed, -lin_y_speed, ang_speed)
        for i, caster in enumerate(self.caster_wheels):
            caster.setPosition(wheel_parameters[i])
            caster.setVelocity(self.robot_caster_velocity)
        for wheel in self.front_left_wheels:
            wheel.setVelocity(wheel_parameters[4])
            
        for wheel in self.front_right_wheels:
            wheel.setVelocity(wheel_parameters[5])
        for wheel in self.back_left_wheels:
            wheel.setVelocity(wheel_parameters[6])
        for wheel in self.back_right_wheels:
            wheel.setVelocity(wheel_parameters[7])

    def set_head_pan(self, data):
        head_final_positions = data.points.pop(0)
        act_rotation = self.head_joint.getField('rotation').getSFRotation()
        final_angle = head_final_positions.positions[0]
        z_sign = final_angle / abs(final_angle)
        # print("ACT ROTATION", act_rotation)
        # print("SET ROTATION", [0.0, 0.0, z_sign, abs(final_angle)])
        # print("REQUIRED ROTATION", [0.0, 0.0, head_final_positions.positions[0]/abs(head_final_positions.positions[0]), abs(head_final_positions.positions[0])])
        self.head_joint.getField('rotation').setSFRotation([0.0, 0.0, z_sign, abs(final_angle)])

    def euler_to_axis_angle(self,euler_angles):
        rotation_matrix = euler2mat(euler_angles[0], euler_angles[1], euler_angles[2])
        axis_angle = mat2axangle(rotation_matrix)
        return axis_angle

    def publish_data(self):
        act_timestamp = rospy.Time.now()
        self.get_camera_image(act_timestamp)
        self.get_pose_data(act_timestamp)        
        self.get_lidar_data(act_timestamp)
        self.update_head_rotation(act_timestamp)

    def cmd_vel_thread(self, event: Event):
        while not event.is_set():
            # self.speed_subscriber = rospy.Subscriber("/joy", Joy, self.set_speed_joy)
            self.speed_subscriber = rospy.Subscriber("/cmd_vel", Twist, self.set_speed)
            rospy.spin()
        
    def __init__(self, enable_joystick, people_number):
        Robot.__init__(self)
        self.enable_joystick = enable_joystick
        self.findAndEnableDevices()
        self.odom_publisher = rospy.Publisher("/odom", Odometry, queue_size=5)
        # self.camera_odom_publisher = rospy.Publisher("/camera_odom", Odometry, queue_size=5)
        self.pose_ground_truth_publisher = rospy.Publisher("/base_pose_ground_truth", Odometry, queue_size=5)
        # self.odom_publisher = rospy.Publisher("/base_pose_ground_truth", Odometry, queue_size=10)
        self.cv_bridge = CvBridge()
        self.camera_color_publisher = rospy.Publisher("/xtion/rgb/image_raw", Image, queue_size=5)
        self.camera_depth_publisher = rospy.Publisher("/xtion/depth/image_raw", Image, queue_size=5)
        self.lidar_publisher = rospy.Publisher("/base_scan", LaserScan, queue_size=5)
        self.head_pan_subscriber = rospy.Subscriber("/head_traj_controller/command", JointTrajectory, self.set_head_pan)
        self.tf_sender = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()

        self.robot_node = self.getFromDef("PR2")
        for i in range(people_number):
            person_node = self.getFromDef("Human"+str(i + 1))
            controller_field = person_node.getField('controller')  
            controller_field.setSFString("<extern>")
        
        # self.camera_pose_respect_robot = list(map(lambda x,y: x-y ,self.camera_node.getPosition(),self.robot_node.getPosition()))
        self.thread_period = 50
        self.event = Event()
        self.vel_thread = Thread(target=self.cmd_vel_thread, args=[self.event],
                                      name="cmd_vel_thread", daemon=True)
        self.vel_thread.start()

if __name__ == '__main__':
    os.environ["WEBOTS_CONTROLLER_URL"] = "ipc://1234/PR2"
    parser = argparse.ArgumentParser()
    parser.add_argument('--joystick', type=int)
    parser.add_argument('--people_number', type=int)
    args, unknown = parser.parse_known_args()

    print("PEROPLE NUMBER", args)

    joystick = args.joystick
    people_number = args.people_number

    enable_joystick = False
    if joystick:
        enable_joystick = True
        print("Joystick setted")
    else:
        print("Joystick not setted")

    if not people_number:
        people_number = 0

    rospy.init_node("PR2_robot")
    rospy.loginfo("PR2_robot node has been started")
    PR2_robot = PR2(enable_joystick, people_number)
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():   
        while PR2_robot.step(PR2_robot.timeStep) != -1:  
            
            PR2_robot.publish_data()
            try:
                rate.sleep()
            except KeyboardInterrupt:
                print("Shutting down")


