import os
import sys
import numpy as np

# Provided simulator code
sys.path.extend(os.path.abspath(os.path.join(os.path.dirname(os.getcwd()),
                                             *["padm_project_2023f", d])) for d in ["", "pddlstream", "ss-pybullet"])

import pybullet_tools.utils as pb
from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

# These are from padm_project_2023f
from src.utils import COUNTERS, compute_surface_aabb, name_from_type


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
# Helper functions from minimal_example.py
def add_ycb(world, ycb_type, counter=0, **kwargs) -> (str, tuple):
    name = name_from_type(ycb_type)
    world.add_body(name, color=np.ones(4))
    pose = pose2d_on_surface(world, name, COUNTERS[counter], **kwargs)
    return name, pose


def pose2d_on_surface(world, entity_name, surface_name, pose2d=(0., 0., 0.)):
    """
    TODO docstring
    pose2d: (x, y, yaw)
    """
    x, y, yaw = pose2d
    body = world.get_body(entity_name)
    print("[pose2d_on_surface] body = ", body)
    surface_aabb = compute_surface_aabb(world, surface_name)
    z = pb.stable_z_on_aabb(body, surface_aabb)
    pose = pb.Pose(pb.Point(x, y, z), pb.Euler(yaw=yaw))
    pb.set_pose(body, pose)
    print("[pose2d_on_surface] entity {} pose = {}".format(entity_name, pose))
    return pose


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

def put_down_sugar():
    """Put sugar down on counter and return new sugar pose"""
    return

def put_down_spam():
    """Put spam down inside drawer and return new spam pose"""