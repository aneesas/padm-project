import os
import sys
import numpy as np

# Local project code
from motion_planning import *

# Provided simulator code, which is not set up to be installed as packages
sys.path.extend(os.path.abspath(os.path.join(os.path.dirname(os.getcwd()),
                                             *["padm_project_2023f", d])) for d in ["", "pddlstream", "ss-pybullet"])

import pybullet_tools.utils as pb_utils
from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

# These are from padm_project_2023f
from src.world import World
from src.helpers import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
    BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
    STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly, \
    SUGAR, SPAM

# Constants
UNIT_POSE2D = (0., 0., 0.)  # x, y, yaw
INIT_POSE_SUGAR = (-0.2, 0.65, np.pi / 4)
INIT_POSE_SPAM = (0.2, 1.1, np.pi / 4)
POSE_OPEN_COUNTER = ()
POSE_DRAWER = ()

# Helper functions from minimal_example.py
def add_ycb(world, ycb_type, counter=0, **kwargs) -> str:
    name = name_from_type(ycb_type)
    world.add_body(name, color=np.ones(4))
    pose2d_on_surface(world, name, COUNTERS[counter], **kwargs)
    return name

def pose2d_on_surface(world, entity_name, surface_name, pose2d=UNIT_POSE2D):
    x, y, yaw = pose2d
    body = world.get_body(entity_name)
    surface_aabb = compute_surface_aabb(world, surface_name)
    z = pb_utils.stable_z_on_aabb(body, surface_aabb)
    pose = pb_utils.Pose(pb_utils.Point(x, y, z), pb_utils.Euler(yaw=yaw))
    pb_utils.set_pose(body, pose)
    return pose

add_sugar_box = lambda world, **kwargs: add_ycb(world, SUGAR, **kwargs)
add_spam_box = lambda world, **kwargs: add_ycb(world, SPAM, **kwargs)

if __name__ == "__main__":
    print("Random seed:", pb_utils.get_random_seed())
    print("Numpy seed:", pb_utils.get_numpy_seed())

    world = World(use_gui=True)

    # Set up simulation world as expected
    # Sugar box on stovetop burner
    # Spam box on countertop
    # TODO(aneesa): I'm not sure counter argument does anything
    #   because the items get placed in the same spot regardless
    sugar_box = add_sugar_box(world, counter=0, pose2d=INIT_POSE_SUGAR)
    spam_box = add_spam_box(world, counter=0, pose2d=INIT_POSE_SPAM)
    print("Sugar box = ", sugar_box)
    print("Spam box = ", spam_box)
    pb_utils.wait_for_user()

    # Move robot forward

    # Move arm to 

    # TODO set up RobotPlanner object that reads in PDDL files and initializes activity + motion planners

