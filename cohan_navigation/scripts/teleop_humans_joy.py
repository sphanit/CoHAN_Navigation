#!/usr/bin/env python
# Brief: Node for controlling human avatars through joy stick
# Author: Phani Teja Singamaneni

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from pynput import keyboard

class TeleopHumans(object):
  def __init__(self, h_id):
    self.h_id = h_id
    self.angular_ = 3
    self.linear_x = 1
    self.linear_y = 0
    rospy.init_node("teleop_humans_joy")
    rospy.Subscriber("/keyword/cmd_vel", Twist, self.JoyCB)

# Collect events until released
    with keyboard.Listener(
            on_press=on_press_method) as listener:
      listener.join()

    rospy.spin()

  def setHumanID(self, id):
    name = '/human'+str(id)+'/cmd_vel'
    self.vel_pub = rospy.Publisher(name, Twist, queue_size = 1)

  def JoyCB(self, msg):
    velocity = Twist()
    velocity.linear.x = msg.linear.x
    velocity.linear.y = msg.linear.y
    velocity.angular.z = msg.angular.z
    self.vel_pub.publish(velocity)

  def on_press_method(key):
      try:
          print('alphanumeric key {0} pressed'.format(
              key.char))
      except AttributeError:
          print('special key {0} pressed'.format(
              key))
        

if __name__ == '__main__':
  teleop_joy = TeleopHumans(1)
