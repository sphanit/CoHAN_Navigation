#!/usr/bin/env python3
import math
import rospy
from nav_msgs.msg import Path, Odometry
from nav_msgs.srv import GetPlan
from std_msgs.msg import Bool
from cohan_gazebo.msg import setGoal, abortGoal
from geometry_msgs.msg import PoseStamped, Point, Quaternion
# from tf.transformations import quaternion_from_euler
import message_filters


class ActorPathPublisher(object):
    def __init__(self, num_agents = 1):
        self.num_agents = num_agents
        self.actors_poses = {}
        rospy.init_node('path_publisher_node')
        rospy.wait_for_service('/move_base/GlobalPlanner/make_plan')
        try:
            self.get_plan = rospy.ServiceProxy('/move_base/GlobalPlanner/make_plan', GetPlan)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        
        agent_sub = []
        self.path_pubs = {}
        self.abort_pubs = {}
        for agent_id in range(1,self.num_agents+1):
            name = 'human'+str(agent_id)
            agent_sub.append(message_filters.Subscriber("/" + name + "/odom", Odometry))
            self.path_pubs[agent_id] = rospy.Publisher("/"+name+"/cmd_path", Path, queue_size=1, latch=True)
            self.abort_pubs[agent_id] = rospy.Publisher("/"+name+"/abort_goal", Bool, queue_size=1, latch=True)

        pose_msg = message_filters.TimeSynchronizer(agent_sub, 10)
        pose_msg.registerCallback(self.odomsCB)
        goal_sub = rospy.Subscriber("/human_goals", setGoal, self.goalCB)
        abort_sub = rospy.Subscriber("/abort_human_goals", abortGoal, self.abortCB)
    
    def odomsCB(self,*msg):
        for i in range(1, self.num_agents+1):
            self.actors_poses[i] = msg[i-1].pose.pose
        
    def goalCB(self, goal_msg):
        self.plan_and_publish_path(goal_msg)
        
    def abortCB(self, msg):
        self.abort_pubs[msg.id].publish(msg.abort)

    def plan_and_publish_path(self, goal_msg):
        actor_id = goal_msg.id
        if(self.actors_poses.get(actor_id) == None):
            rospy.logwarn("ACtor's start pose cannot be found for id = %d", actor_id)
        start_pose = self.actors_poses[actor_id]
        goal_pose = goal_msg.goal.pose
        now = rospy.Time.now()
        start = PoseStamped()
        goal = PoseStamped()
        start.header.frame_id = "map"
        start.header.stamp = now
        start.pose = start_pose
        goal.header.frame_id = "map"
        goal.header.stamp = now
        goal.pose = goal_pose
        tolerance = .5
        
        # Call the service to get the plan
        response = self.get_plan(start, goal, tolerance)

        if len(response.plan.poses) > 0:
            self.path_pubs[goal_msg.id].publish(response.plan)
            # print ('published path', response.plan)
        else:
            rospy.logwarn("Plan cannot be found for actor id = %d", actor_id)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        actor_path_pub = ActorPathPublisher(2)
        actor_path_pub.run()
    except rospy.ROSInterruptException:
        pass
