#!/usr/bin/python3

# Brief: This node subscribes to the robots published on /humani, i=1,2, .. and robot, and publishes /tracked_agents required for CoHAN
# Author: Phani Teja Singamaneni

import sys
import rospy
import time
from cohan_msgs.msg import TrackedAgents, TrackedAgent, TrackedSegment, TrackedSegmentType, AgentType
from yolov8_data.msg import ObjectsMSG
from nav_msgs.msg import Odometry
import message_filters
import math


class WebotsAgents(object):

    def __init__(self):
        self.tracked_agents_pub = []
        self.Segment_Type = TrackedSegmentType.TORSO
        self.agents = TrackedAgents()
        self.robot = TrackedAgent()
        self.sig_1 = False
        self.sig_2 = False

    def AgentsPub(self):
        rospy.init_node('Webots_Agents', anonymous=True)
        agent_sub = []

        # Subscibe to human agents
        self.sig_1 = True


        people_sub = rospy.Subscriber("/tracked_people", ObjectsMSG, self.AgentsCB)

        # Subscribe to the robot
        # if self.ns != "":
        # robot_sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, self.RobotCB)
        # else:
        self.sig_2 = True

        self.tracked_agents_pub = rospy.Publisher("tracked_agents", TrackedAgents, queue_size=1)
        # pose_msg = message_filters.TimeSynchronizer(agent_sub, 10)
        # pose_msg.registerCallback(self.AgentsCB)
        rospy.Timer(rospy.Duration(0.033), self.publishAgents)
        rospy.spin()

    def AgentsCB(self,msg):
        tracked_agents = TrackedAgents()
        for agent in msg.objectsmsg:
            if not agent.exist_position:
                continue
            agent_segment = TrackedSegment()
            agent_segment.type = self.Segment_Type
            agent_segment.pose.pose = agent.pose.pose
            agent_segment.twist.twist = agent.speed                
            tracked_agent = TrackedAgent()
            tracked_agent.type = AgentType.HUMAN
            tracked_agent.name = "human"+str(agent.id)
            tracked_agent.track_id = agent.id
            tracked_agent.segments.append(agent_segment)
            tracked_agents.agents.append(tracked_agent)
        # if tracked_agents.agents:
        self.agents = tracked_agents
        self.sig_1 = True

    def RobotCB(self, msg):
        agent_segment = TrackedSegment()
        agent_segment.type = self.Segment_Type
        agent_segment.pose.pose = msg.pose.pose
        agent_segment.twist.twist = msg.twist.twist
        tracked_agent = TrackedAgent()
        tracked_agent.type = AgentType.ROBOT
        tracked_agent.track_id = 0
        tracked_agent.name = "robot"
        tracked_agent.segments.append(agent_segment)
        self.robot = tracked_agent
        self.sig_2 = True

    def publishAgents(self, event):
        if self.sig_1:
            self.agents.header.stamp = rospy.Time.now()
            self.agents.header.frame_id = "map"
            self.tracked_agents_pub.publish(self.agents)
            self.sig_1 = False

if __name__ == '__main__':
    agents = WebotsAgents()
    agents.AgentsPub()
