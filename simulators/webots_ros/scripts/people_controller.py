from controller import *
from bvh_webots import WebotsBVH

import sys
sys.path.append('/home/robocomp/software/npybvh')
from bvh import Bvh

import numpy as np
from math import cos, sin, radians

class Person (Supervisor):
    def __init__(self):
        Robot.__init__(self)
        self.timeStep = int(self.getBasicTimeStep())    

        # Load bvh
        self.bvh = WebotsBVH()
        self.bvh.read_file()
        self.bvh.get_pose_data()

        # Get person skin       
        self.get_skin_data()

        # Find correspondencies between the Skin's bones and BVH's joint.
        self.index_skin_to_bvh = [-1] * self.bone_count
        print("finding dependences")
        for i in range(self.bone_count):
            if i == 24 or i == 25 or i == 26 or i == 15 or i == 16 or i == 17:
                continue
            skin_name = self.joint_name_list[i]
            for j in range(self.bvh.get_joint_count()):
                if skin_name == self.bvh.get_joint_name(j):
                    self.index_skin_to_bvh[i] = j
                    break
            print(self.bvh.get_joint_name(j), "Skin bone", i, "corresponds to", self.index_skin_to_bvh[i], "bvh joint")

        # Pass absolute and relative joint T pose orientation
        for i in range(self.bone_count):
            if self.index_skin_to_bvh[i] < 0:
                continue  
            self.bvh.set_global_t_pose(self.index_skin_to_bvh[i], self.skin.getBoneOrientation(i, True)) 
            self.bvh.set_local_t_pose(self.index_skin_to_bvh[i], self.skin.getBoneOrientation(i, False))

        self.initial_root_position = [0, 0, 0]
        self.root_position_offset = [0, 0, 0]
        self.skin_root_position = self.skin.getBonePosition(self.root_bone_index, False)
        if self.root_bone_index >= 0:
            self.current_root_position = self.bvh.get_pose_value(0, 0)
            # print("self.current_root_position", self.current_root_position)
            for i in range(3):
                self.root_position_offset[i] = self.skin_root_position[i] - self.current_root_position[i]
                self.initial_root_position[i] = self.current_root_position[i]


    def load_bvh_data(self):
        self.anim = Bvh()
        self.anim.parse_file('walk.bvh')
        self.all_poses, self.all_rotations = self.anim.all_frame_poses()
        self.joint_names = list(self.anim.joint_names())
        self.joint_count = len(self.joint_names)
        self.frame_count = len(self.all_poses)
        self.scale_factor = 20

    def get_skin_data(self):
        self.skin_name = "skin"
        self.skin = self.getDevice(self.skin_name)
        self.bone_count = self.skin.getBoneCount()
        print("self.bone_count", self.bone_count)
        self.root_bone_index = -1
        self.joint_name_list = [None] * self.bone_count

        # Get bone names in the skin

        print("GETTING BONES NAMES")
        for i in range(self.bone_count):
            name = self.skin.getBoneName(i)
            self.joint_name_list[i] = name
            print(i, name)
            if name == "Hips":
                self.root_bone_index = i
        print("joint_name_list", self.joint_name_list)
        
    def get_joint_rotation(self, joint):
        return self.bvh.get_joint_rotation(joint)

    # def set_joint_rotation(self):
    #     for i in range(self.bone_count):
    #         if self.index_skin_to_bvh[i] < 0:
    #             continue
    #         joint_orientation = self.euler_to_quaternion(self.all_rotations[self.index_skin_to_bvh[i]])
    #         self.skin.setBoneOrientation(i, joint_orientation, False)

    # def fetch_next_frame(self):
    #     self.remaining_frames = self.end_frame_index - self.current_frame
    #     if self.remaining_frames < 4:
    #         if self.root_bone_index >= 0:
    #             self.current_frame =



    # def offset_position(self):






if __name__ == '__main__':
    person = Person()       
    while person.step(person.timeStep) != -1:  
        # print(person.bvh.get_current_frame())
        for i in range(person.bone_count):
            if person.index_skin_to_bvh[i] < 0:
                continue
            orientation = person.get_joint_rotation(person.index_skin_to_bvh[i])
            # if person.bvh.get_current_frame() == 4:
            #     print(round(orientation[0], 6), round(orientation[1], 6), round(orientation[2], 6), round(orientation[3], 6), i)
            person.skin.setBoneOrientation(i, orientation, False)
        # if person.bvh.get_current_frame() == 4:    
        #     exit()
        person.bvh.set_next_frame()