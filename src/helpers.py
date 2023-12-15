import os
import sys

# Provided simulator code
sys.path.extend(os.path.abspath(os.path.join(os.path.dirname(os.getcwd()),
                                             *["padm_project_2023f", d])) for d in ["", "pddlstream", "ss-pybullet"])

import pybullet_tools.utils as pb
from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics


### Motion planning helpers
class Node():
    """Simple node to keep track of 3-DOF pose and parentage"""
    def __init__(self, pose: tuple, parent: object=None):
        self._pose = pose
        self._parent = parent
    
    def set_parent(self, parent):
        self._parent = parent
    
    @property
    def pose(self):
        return self._pose
    
    @property
    def parent(self):
        return self._parent


### Simulation helpers
def move_arm(world, link, path: list):
    if len(path) == 0:
        print("[move_arm] WARNING: Got empty path!")
        return

    start_pose = pb.get_link_pose(world.robot, link)
    ik_joints = get_ik_joints(world.robot, PANDA_INFO, link)
    prev_pose = start_pose

    print("\nMoving through path...")
    for pose in path:
        interpolated_poses = pb.interpolate_poses(prev_pose, pose, pos_step_size=0.1)
        for p in interpolated_poses:
            conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, link, p, max_time=0.05), None)
            if conf is None:
                print("Something went wrong!!")
                break
            pb.set_joint_positions(world.robot, ik_joints, conf)
        prev_pose = pose

    return