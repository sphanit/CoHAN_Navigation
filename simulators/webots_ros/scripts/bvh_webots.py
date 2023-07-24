import sys
sys.path.append('/home/robocomp/software/npybvh')
from bvh import Bvh

from quaternions import Quaternions

class WebotsBVH ():
    def __init__(self):
        # Get bvh files
        self.anim = Bvh()
        self.quaternion = Quaternions()
        self.poses, self.rotations, self.joint_list = [], [], []
        self.joint_count, self.frame_count = 0, 0 
        self.scale_factor = 20

        self.current_frame = 0
        self.remaining_frames = -1
        self.end_frame_index = -1

        self.global_skin_bone_orientations = []
        self.local_skin_bone_orientations = []
        self.bvh_t_pose = []

    def read_file(self):
        self.anim.parse_file('walk.bvh')

    def get_pose_data(self):
        self.poses, rotations_ = self.anim.all_frame_poses()
        
        self.joint_list = list(self.anim.joint_names())
        self.joint_count = len(self.joint_list)
        self.frame_count = len(self.poses)
        self.remaining_frames = self.frame_count - 1
        self.end_frame_index = self.get_frame_count() - 1
        self.global_skin_bone_orientations = [-1] * self.joint_count
        self.local_skin_bone_orientations = [-1] * self.joint_count
        for i in range(self.frame_count):
            joint = []
            bvh_t_pose = []
            for j in range(self.joint_count):
                bvh_t_pose.append(self.quaternion.quaternion_zero())
                if i == 4 and j == 1:
                    print(rotations_[i][j])
                    exit()
                joint.append(self.quaternion.euler_to_quaternion(rotations_[i][j]))
            self.rotations.append(joint)
            self.bvh_t_pose.append(bvh_t_pose)

    def get_pose_value(self, joint, frame):
        return self.poses[frame][joint]
    
    def get_rotation_value(self, joint, frame):
        return self.rotations[frame][joint]

    def get_joint_name(self, index):
        return self.joint_list[index]
    
    def get_joint_count(self):
        return self.joint_count
    
    def get_frame_count(self):
        return self.frame_count
    
    def get_current_frame(self):
        return self.current_frame
    
    def get_end_frame(self):
        return self.end_frame_index
    
    def set_next_frame(self):
        self.current_frame += 4

    def set_global_t_pose(self, index, pose):
        self.global_skin_bone_orientations[index] = self.quaternion.axis_angle_to_quaternion([pose[0], pose[1], pose[2]], pose[3])

    def set_local_t_pose(self, index, pose):
        # print("INDEX set_local_t_pose:", index)
        # print(pose[0], pose[1], pose[2], pose[3])
        self.local_skin_bone_orientations[index] = self.quaternion.axis_angle_to_quaternion([pose[0], pose[1], pose[2]], pose[3])
        # print("set_local_t_pose", self.local_skin_bone_orientations[index])

    def get_global_t_pose(self, index):
        return self.global_skin_bone_orientations[index]

    def get_local_t_pose(self, index):
        # print("INDEX get_local_t_pose:", index)
        # print("get_local_t_pose", self.local_skin_bone_orientations[index])
        return self.local_skin_bone_orientations[index]

    def set_scale_factor(self, factor):
        self.scale_factor = factor
 
    def get_joint_rotation(self, index):
        act_rotation = self.get_rotation_value(index, self.get_current_frame())   
        if self.get_current_frame() == 4 and index == 0: 
            print(act_rotation)
        act_rotation = self.quaternion.normalize_quaternion(act_rotation)   
        if self.get_current_frame() == 4 and index == 0: 
            print(act_rotation)
        if self.get_current_frame() == 0:
            self.bvh_t_pose[self.current_frame][index] = act_rotation
            return self.quaternion.quaternion_to_axis_angle(self.get_local_t_pose(index))
                

        act_rotation = self.quaternion.multiply_quaternions(self.quaternion.conjugate_quaternion(self.bvh_t_pose[self.current_frame][index]), act_rotation)
        
        # Convert rotation axis to Webots bone T pose coordinate system
        result = self.quaternion.quaternion_to_axis_angle(act_rotation)
        axis = [result[0], result[1], result[2], 0]
        angle = result[3]
        g = self.get_global_t_pose(index)
        axis = self.quaternion.multiply_quaternions(axis, g)
        axis = self.quaternion.multiply_quaternions(self.quaternion.conjugate_quaternion(g), axis)
        result = self.quaternion.quaternion_to_axis_angle(axis)

        # Add converted frame rotation to Webots bone T pose rotation

        frame_rotation = self.quaternion.axis_angle_to_quaternion([result[0], result[1], result[2]], angle)
        frame_rotation = self.quaternion.multiply_quaternions(self.get_local_t_pose(index), frame_rotation)
        return self.quaternion.quaternion_to_axis_angle(frame_rotation)