#!/usr/bin/python3

# Brief: This node subscribes to the robots published on /humani, i=1,2, .. and robot, and publishes /tracked_agents required for CoHAN
# Author: Phani Teja Singamaneni

import sys
import rospy
import time
import math
from cohan_msgs.msg import TrackedAgents, TrackedAgent, TrackedSegment, TrackedSegmentType, AgentType
from geometry_msgs.msg import Twist
import tf2_ros
from tf.transformations import *

class MultiverseAgents(object):
    def __init__(self):
        rospy.init_node('multiverse_Agents', anonymous=True)
        self.tracked_agents_pub = []
        self.Segment_Type = TrackedSegmentType.TORSO
        self.agents = TrackedAgents()
        self.poses = {}
        self.velocities = {}
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.rate = rospy.Rate(10)
        self.tracked_agents_pub = rospy.Publisher("/tracked_agents", TrackedAgents, queue_size=1)
        rospy.Timer(rospy.Duration(0.02), self.agentsPub)
        self.last_time = rospy.Time.now()
        rospy.spin()

    def agentsPub(self, event):
        try:
            human_pose = self.tfBuffer.lookup_transform("map", "CharacterMesh0", rospy.Time(0))
            tracked_agents = TrackedAgents()
            agent_segment = TrackedSegment()
            agent_segment.type = self.Segment_Type
            agent_segment.pose.pose.position.x = human_pose.transform.translation.x
            agent_segment.pose.pose.position.y = human_pose.transform.translation.y
            r,p,y = euler_from_quaternion([human_pose.transform.rotation.x, human_pose.transform.rotation.y, human_pose.transform.rotation.z, human_pose.transform.rotation.w])
            q = quaternion_from_euler(y-1.57, 0, 0, 'rzyx')
            agent_segment.pose.pose.orientation.x = q[0]
            agent_segment.pose.pose.orientation.y = q[1]
            agent_segment.pose.pose.orientation.z = q[2]
            agent_segment.pose.pose.orientation.w = q[3]
            if(self.velocities):
                agent_segment.twist.twist = self.velocities[1]
            tracked_agent = TrackedAgent()
            tracked_agent.type = AgentType.HUMAN
            tracked_agent.name = "human1"
            tracked_agent.track_id = 1
            tracked_agent.segments.append(agent_segment)
            tracked_agents.agents.append(tracked_agent)
            if(tracked_agents.agents):
                self.agents = tracked_agents
            self.agents.header.stamp = rospy.Time.now()
            self.agents.header.frame_id = "map"
            self.velocities = self.getVelocity()
            self.tracked_agents_pub.publish(self.agents)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.rate.sleep()

    def getVelocity(self):
        velocity = {}
        if(not self.poses):
            for agent in self.agents.agents:
                self.poses[agent.track_id] = agent.segments[0].pose.pose
            return None
    
        for agent in self.agents.agents:
            vel = Twist()
            pose1 = agent.segments[0].pose.pose
            pose2 = self.poses[agent.track_id]
            _,_,yaw1 = euler_from_quaternion([pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w])
            _,_,yaw2 = euler_from_quaternion([pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w])
            dt = (rospy.Time.now() - self.last_time).to_sec()
            ang_diff = self.getNormalizedAngle(yaw1-yaw2)

            vel.linear.x = (pose1.position.x - pose2.position.x)/dt
            vel.linear.y = (pose1.position.y - pose2.position.y)/dt
            vel.angular.z = ang_diff/dt
            # Needs to add filtering to fluctuate less (moving average or kalman)
            velocity[agent.track_id] = vel
            self.poses[agent.track_id] = agent.segments[0].pose.pose

        self.last_time = rospy.Time.now()
        return velocity

    def getNormalizedAngle(self,ang):
        ang = ang % (2*math.pi)
        if ang > math.pi:
            ang -= 2 * math.pi
        elif ang < -math.pi:
            ang += 2 * math.pi
        return ang





        

if __name__ == '__main__':
    agents = MultiverseAgents()
