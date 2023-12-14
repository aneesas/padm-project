import os
import sys
import numpy as np
import time

# Local project code
# from motion_planning import rrt

# Provided simulator code, which is not set up to be installed as packages
sys.path.extend(os.path.abspath(os.path.join(os.path.dirname(os.getcwd()),
                                             *["padm_project_2023f", d])) for d in ["", "pddlstream", "ss-pybullet"])

import pybullet_tools.utils as pb
from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

# These are from padm_project_2023f
from src.world import World
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
    BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
    STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly, \
    SUGAR, SPAM

# Constants
UNIT_POSE2D = (0., 0., 0.)  # x, y, yaw
INIT_POSE_SUGAR = (-0.2, 0.65, np.pi / 4)  # x, y, yaw in world
INIT_POSE_SPAM = (0.2, 1.1, np.pi / 4)  # x, y, yaw in world
POSE_OPEN_COUNTER = ()
POSE_DRAWER = ()

# Helper functions from minimal_example.py
def add_ycb(world, ycb_type, counter=0, **kwargs) -> (str, tuple):
    name = name_from_type(ycb_type)
    world.add_body(name, color=np.ones(4))
    pose = pose2d_on_surface(world, name, COUNTERS[counter], **kwargs)
    return name, pose

def pose2d_on_surface(world, entity_name, surface_name, pose2d=UNIT_POSE2D):
    x, y, yaw = pose2d
    body = world.get_body(entity_name)
    print("[pose2d_on_surface] body = ", body)
    surface_aabb = compute_surface_aabb(world, surface_name)
    z = pb.stable_z_on_aabb(body, surface_aabb)
    pose = pb.Pose(pb.Point(x, y, z), pb.Euler(yaw=yaw))
    pb.set_pose(body, pose)
    print("[pose2d_on_surface] entity {} pose = {}".format(entity_name, pose))
    return pose

def get_sample_fn(body, joints, custom_limits={}, **kwargs):
    lower_limits, upper_limits = pb.get_custom_limits(body, joints, custom_limits,
                                                      circular_limits=pb.CIRCULAR_LIMITS)
    generator = pb.interval_generator(lower_limits, upper_limits, **kwargs)
    def fn():
        return tuple(next(generator))
    return fn

add_sugar_box = lambda world, **kwargs: add_ycb(world, SUGAR, **kwargs)
add_spam_box = lambda world, **kwargs: add_ycb(world, SPAM, **kwargs)

if __name__ == "__main__":
    print("Random seed:", pb.get_random_seed())
    print("Numpy seed:", pb.get_numpy_seed())

    world = World(use_gui=True)

    # Scoot robot over for better starting position
    init_base_pos = pb.get_joint_positions(world.robot, world.base_joints)
    # Using world coordinate frame, scoot in y-pos direction
    new_base_pos = np.array(init_base_pos) + np.array([0.0, 0.4, 0.0])
    pb.set_joint_positions(world.robot, world.base_joints, new_base_pos)

    trajectory = np.array([
    [1.20015843e-02, -9.67484157e-02, -2.05498416e-01, -2.07680280e-01, -2.07680280e-01, -2.07680280e-01, -2.07680280e-01, -2.07680280e-01, -2.07680280e-01, -2.07680280e-01, -2.07680280e-01, -2.07680280e-01, -2.07680280e-01, -2.07680280e-01, -2.07680280e-01],
    [-5.69781601e-01, -4.61031601e-01, -3.52281601e-01, -2.43531601e-01, -1.34781601e-01, -2.60316014e-02, 8.27183986e-02, 1.91468399e-01, 2.14383836e-01, 2.14383836e-01, 2.14383836e-01, 2.14383836e-01, 2.14383836e-01, 2.14383836e-01, 2.14383836e-01],
    [5.68014875e-05, 1.08806801e-01, 2.17556801e-01, 3.19789801e-01, 3.19789801e-01, 3.19789801e-01, 3.19789801e-01, 3.19789801e-01, 3.19789801e-01, 3.19789801e-01, 3.19789801e-01, 3.19789801e-01, 3.19789801e-01, 3.19789801e-01, 3.19789801e-01],
    [-2.81059694e+00, -2.68009694e+00, -2.54959694e+00, -2.41909694e+00, -2.28859694e+00, -2.15809694e+00, -2.02759694e+00, -1.89709694e+00, -1.76659694e+00, -1.63609694e+00, -1.50559694e+00, -1.37509694e+00, -1.24459694e+00, -1.11409694e+00, -1.04157275e+00],
    [-2.57683743e-04, -1.30757684e-01, -1.27232139e-01, -1.27232139e-01, -1.27232139e-01, -1.27232139e-01, -1.27232139e-01, -1.27232139e-01, -1.27232139e-01, -1.27232139e-01, -1.27232139e-01, -1.27232139e-01, -1.27232139e-01, -1.27232139e-01, -1.27232139e-01],
    [3.03634501e+00, 2.90584501e+00, 2.77534501e+00, 2.64484501e+00, 2.51434501e+00, 2.38384501e+00, 2.25334501e+00, 2.12284501e+00, 2.03978070e+00, 2.03978070e+00, 2.03978070e+00, 2.03978070e+00, 2.03978070e+00, 2.03978070e+00, 2.03978070e+00],
    [7.41070151e-01, 8.15890203e-01, 8.15890203e-01, 8.15890203e-01, 8.15890203e-01, 8.15890203e-01, 8.15890203e-01, 8.15890203e-01, 8.15890203e-01, 8.15890203e-01, 8.15890203e-01, 8.15890203e-01, 8.15890203e-01, 8.15890203e-01, 8.15890203e-01]
]   )

    tool_link = pb.link_from_name(world.robot, "panda_hand")
    pb.wait_for_user()

    for t in range(len(trajectory[0])):
        joint_values = [trajectory[j, t] for j in range(len(trajectory))]
        pb.set_joint_positions(world.robot, world.arm_joints, joint_values)
        time.sleep(0.1)

    print("Done!")
    pb.wait_for_user()