from controller import Camera, Supervisor, Robot, RangeFinder, Lidar, Motor
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

        # Camera RGBD
        self.camera_color = self.getDevice("camera_color")
        self.camera_color.enable(33)
        self.camera_depth = self.getDevice("camera_depth")
        self.camera_depth.enable(33)

        # Head joints
        self.head_1 = self.getDevice("head_1_joint")
        self.head_2 = self.getDevice("head_2_joint")
        self.head_1.setPosition(0)
        self.head_2.setPosition(0)

        # Torso joints
        self.torso_lift_joint = self.getDevice("torso_lift_joint")
        self.torso_lift_joint.setPosition(0)

        # Wheels
        self.wheel_left = self.getDevice("wheel_left_joint")
        self.wheel_right = self.getDevice("wheel_right_joint")
        self.wheel_left.setPosition(float('inf'))
        self.wheel_right.setPosition(float('inf'))
        self.wheel_left.setVelocity(float(0))
        self.wheel_right.setVelocity(float(0))

        # LIDAR
        self.lidar = self.getDevice("Hokuyo URG-04LX-UG01")
        self.lidar.enable(100)
        self.lidar.enablePointCloud()

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

        # color = cv2.cvtColor(np.frombuffer(color, np.uint8).reshape(self.camera_color.getHeight(), self.camera_color.getWidth(), 4), cv2.COLOR_RGBA2RGB )
        # depth = np.ctypeslib.as_array(depth, (self.camera_depth.getWidth() * self.camera_depth.getHeight(),)).reshape(self.camera_depth.getHeight(), self.camera_depth.getWidth(), 1)

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
            if lidar_range[i] == float('inf'):
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
        yaw_to_euler = tf.transformations.quaternion_from_euler(0, 0, 0)       
        self.tf_sender.sendTransform((0, 0, 0), yaw_to_euler, laser_publish_data.header.stamp, "base_laser_link", "base_link")

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
        yaw_to_euler = tf.transformations.quaternion_from_euler(0, 0, math.atan2(robot_orientation[0], robot_orientation[1])-math.pi/2)       
        odometry_message.pose.pose.orientation = Quaternion(x=yaw_to_euler[0], y=yaw_to_euler[1], z=yaw_to_euler[2], w=yaw_to_euler[3])
        odometry_message.twist.twist.linear.x = round(robot_speed[0], 3)
        odometry_message.twist.twist.linear.y = round(robot_speed[1], 3)
        odometry_message.twist.twist.angular.z = round(robot_speed[5], 3)
        # self.tf_sender.sendTransform((robot_pose_ground_truth[0], robot_pose_ground_truth[1], 0), yaw_to_euler, rospy.Time.now(), "base_link", "odom")
        self.tf_sender.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0), timestamp, "base_footprint", "base_link")
        self.tf_sender.sendTransform((robot_pose_ground_truth[0], robot_pose_ground_truth[1], 0), yaw_to_euler, timestamp, "base_link", "odom")
        self.odom_publisher.publish(odometry_message)
        odometry_message.header.frame_id = "odom"
        self.pose_ground_truth_publisher.publish(odometry_message)
        
    def set_speed(self, data):
        lin_speed = data.linear.x
        ang_speed = data.angular.z
        left, right = self.speed_commands_to_differential_drive(lin_speed, ang_speed)
        # print("LEFT SPEED:", left, "right SPEED:", right)
        self.wheel_left.setVelocity(left)
        self.wheel_right.setVelocity(right)

    def set_speed_joy(self, data):
        lin_speed = data.axes[4]
        ang_speed = data.axes[3]
        left, right = self.speed_commands_to_differential_drive(lin_speed, ang_speed)
        # print("LEFT SPEED:", left, "right SPEED:", right)
        self.wheel_left.setVelocity(left)
        self.wheel_right.setVelocity(right)
    
    def speed_commands_to_differential_drive(self, lin_speed, ang_speed, L=0.5, r=0.1):
        return (lin_speed - (L/2)*ang_speed)/r, (lin_speed + (L/2)*ang_speed)/r

    def publish_data(self):
        act_timestamp = rospy.Time.now()
        self.get_camera_image(act_timestamp)
        self.get_pose_data(act_timestamp)        
        self.get_lidar_data(act_timestamp)

    def __init__(self):
        Robot.__init__(self)
        self.findAndEnableDevices()
        self.odom_publisher = rospy.Publisher("/odom", Odometry, queue_size=5)
        self.pose_ground_truth_publisher = rospy.Publisher("/base_pose_ground_truth", Odometry, queue_size=5)
        # self.odom_publisher = rospy.Publisher("/base_pose_ground_truth", Odometry, queue_size=10)
        self.camera_color_publisher = rospy.Publisher("/xtion/rgb/image_raw", Image, queue_size=5)
        self.camera_depth_publisher = rospy.Publisher("/xtion/depth/image_raw", Image, queue_size=5)
        self.lidar_publisher = rospy.Publisher("/base_scan", LaserScan, queue_size=5)
        self.tf_sender = tf.TransformBroadcaster()
        # self.speed_subscriber = rospy.Subscriber("/cmd_vel", Twist, self.set_speed)
        self.speed_subscriber = rospy.Subscriber("/joy", Joy, self.set_speed_joy)
        self.robot_node = self.getFromDef("TIAGO")

if __name__ == '__main__':
    rospy.init_node("tiago_robot")
    rospy.loginfo("tiago_robot node has been started")
    tiago_robot = Tiago()
    rate = rospy.Rate(25)

    while not rospy.is_shutdown():   
        while tiago_robot.step(tiago_robot.timeStep) != -1:  
            tiago_robot.publish_data()
            try:
                rate.sleep()
            except KeyboardInterrupt:
                print("Shutting down")
    # laser_publisher = rospy.Publisher("/yolo_objects", LaserScan, queue_size=10)
    # color_image_publisher = rospy.Publisher("/camera/color/image_rect_raw", ObjectsMSG, queue_size=10)
    # depth_image_publisher = rospy.Publisher("/camera/depth/image_rect_raw", ObjectsMSG, queue_size=10)

