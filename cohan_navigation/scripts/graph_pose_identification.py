import rospy
import cv2
import numpy as np
import json
import time
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *

class Graph():
    def __init__(self):

        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 1
        self.color = (0, 255, 0)  # Color en formato BGR
        self.thickness = 1  # Grosor del texto

        self.rooms = {}

        self.map_image = None
        self.load_json_graph()
        self.load_map_image()
        self.draw_map_image(self.rooms)

        self.robot_pose = None
        self.person_pose = None
        self.is_robot_pose = False
        self.is_person_pose = False

        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.set_robot_pose)
        self.followed_person_pose_subscriber = rospy.Subscriber("/followed_person_pose", PoseStamped, self.set_person_pose)

    def set_robot_pose(self, data):
        self.robot_pose = data.pose.pose.position
        self.is_robot_pose = True

    def set_person_pose(self, data):
        self.person_pose = data.pose.position
        self.is_person_pose = True

    def show_map(self, event):
        cv2.imshow("MAP", self.map_image)
        cv2.waitKey(1)
            
    def check_point_inside_rectangle(self, point, rectangle):
        x, y = point
        left, top, right, bot = rectangle
        if left <= x <= right and top >= y >= bot:
            return True
        else:
            return False

    def check_robot_pose_in_graph(self, event):
        robot_found, person_found = True, True
        robot_pose, person_pose = None, None
        if self.is_robot_pose:
            robot_found = False
            robot_pose = self.robot_pose
        if self.is_person_pose:
            person_found = False
            person_pose = self.person_pose
        act_nodes = self.rooms
        nodes_keys = list(act_nodes.keys())
        for node in nodes_keys:
            act_polygon = act_nodes[node]["world_polygon"]
            if not robot_found:
                if self.check_point_inside_rectangle([robot_pose.x, robot_pose.y], act_polygon):
                    print("ROBOT IN ROOM", node)
                    robot_found = True
            if not person_found:
                if self.check_point_inside_rectangle([person_pose.x, person_pose.y], act_polygon):
                    print("PERSON IN ROOM", node)
                    person_found = True
            if robot_found and person_found:
                return
            
        self.is_robot_pose = False
        self.is_person_pose = False

    def load_json_graph(self):
        file_name = "laas_adream_graph.json"
        with open(file_name, "r") as file:
            self.rooms = json.load(file)
        print("Data loaded")

    def load_map_image(self):
        image_file = "laas_adream.pgm"
        self.map_image = cv2.imread(image_file, cv2.IMREAD_GRAYSCALE)
        self.map_image = cv2.cvtColor(self.map_image, cv2.COLOR_GRAY2RGB)
        print("Map loaded")

    def draw_map_image(self, graph):
        nodes_keys = list(graph.keys())
        for node in nodes_keys:
            act_node = graph[node]
            room_polygon = act_node["pixel_polygon"]
            cv2.rectangle(self.map_image, (room_polygon[0], room_polygon[1]), (room_polygon[2], room_polygon[3]), (0, 0, 255), 2)
            text_size = cv2.getTextSize(str(node), self.font, self.font_scale, self.thickness)[0]
            cv2.putText(self.map_image, str(node), (int((room_polygon[2] - room_polygon[0])/2 + room_polygon[0] - text_size[0]/2) , int((room_polygon[3] - room_polygon[1])/2 + room_polygon[1] + text_size[1]/2) ), self.font, self.font_scale, self.color, self.thickness)
            edges_list = list(act_node["doors"].keys())
            for edge in edges_list:
                door_point = act_node["doors"][edge]["pixel_pose"]
                cv2.circle(self.map_image, (int(door_point[0]), int(door_point[1])), 3, (255, 0, 0), -1)


if __name__ == '__main__':    
    rospy.init_node("World_graph")
    rospy.loginfo("World graph node has been started")
    
    graph = Graph()
    rospy.Timer(rospy.Duration(0.1), graph.check_robot_pose_in_graph)
    rospy.Timer(rospy.Duration(0.1), graph.show_map)
    rospy.spin()