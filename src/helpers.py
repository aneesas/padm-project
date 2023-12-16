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
from src.utils import COUNTERS, compute_surface_aabb, name_from_type, SUGAR, SPAM


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
def add_ycb(world, ycb_type, counter=0, **kwargs) -> (str, pb.Pose):
    name = name_from_type(ycb_type)
    world.add_body(name, color=np.ones(4))
    pose = pose2d_on_surface(world, name, COUNTERS[counter], **kwargs)
    return name, pose


def pose2d_on_surface(world, entity_name: str, surface_name: str, pose2d=(0., 0., 0.)) -> pb.Pose:
    """
    Creates a 3D pose for the provided object with the z stable on the provided surface.

    pose2d: (x, y, yaw)
    """
    x, y, yaw = pose2d
    body = world.get_body(entity_name)
    surface_aabb = compute_surface_aabb(world, surface_name)
    z = pb.stable_z_on_aabb(body, surface_aabb)
    pose = pb.Pose(pb.Point(x, y, z), pb.Euler(yaw=yaw))
    pb.set_pose(body, pose)
    print("[pose2d_on_surface] entity {} pose = {}".format(entity_name, pose))
    return pose


def move_arm(world, link, path: list):
    """
    Takes a path of poses as a list and uses robot arm inverse kinematics to move the end
    effector through the list of poses. Requires poses to be valid within constraints of
    robot arm kinematics.
    
    link (int): link object for end effector
    """
    if len(path) == 0:
        print("[move_arm] WARNING: Got empty path!")
        return

    start_pose = pb.get_link_pose(world.robot, link)
    ik_joints = get_ik_joints(world.robot, PANDA_INFO, link)
    prev_pose = start_pose

    print("\n[move_arm] Moving through path...")
    for pose in path:
        interpolated_poses = pb.interpolate_poses(prev_pose, pose, pos_step_size=0.1)
        for p in interpolated_poses:
            conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, link, p, max_time=0.05), None)
            if conf is None:
                print("[move_arm] WARNING: Can't move to desired pose!! ", p)
                break
            pb.set_joint_positions(world.robot, ik_joints, conf)
        prev_pose = pose

    return

def put_down_sugar(world, point_3d, surface_name="indigo_tmp"):
    """Put sugar down on counter and return new sugar pose"""
    x, y, _ = point_3d
    pose = pose2d_on_surface(world, SUGAR, surface_name, pose2d=(x, y, np.pi / 4))
    return pose

def put_down_spam(world, drawer_name="indigo_drawer_top"):
    """Put spam down inside drawer and return new spam pose"""
    link = pb.link_from_name(world.kitchen, drawer_name)
    aabb = pb.get_aabb(world.kitchen, link)
    x, y, _ = pb.get_aabb_center(aabb)
    z = aabb[0][2]  # lower limits, z
    body = world.get_body(SPAM)
    pose = pb.Pose(pb.Point(x, y, z), pb.Euler(yaw=np.pi / 4))
    pb.set_pose(body, pose)
    return pose