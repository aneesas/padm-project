import os
import sys
import numpy as np
import time

# Local project code
# from motion_planning import *

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
    z = pb.stable_z_on_aabb(body, surface_aabb)
    pose = pb.Pose(pb.Point(x, y, z), pb.Euler(yaw=yaw))
    pb.set_pose(body, pose)
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

    # Set up simulation world as expected
    # Sugar box on stovetop burner
    # Spam box on countertop
    # TODO(aneesa): I'm not sure counter argument does anything
    #   because the items get placed in the same spot regardless
    sugar_box = add_sugar_box(world, counter=0, pose2d=INIT_POSE_SUGAR)
    spam_box = add_spam_box(world, counter=0, pose2d=INIT_POSE_SPAM)
    print("Sugar box = ", sugar_box)
    print("Spam box = ", spam_box)
    pb.wait_for_user()

    # Scoot robot over for better starting position
    init_base_pos = pb.get_joint_positions(world.robot, world.base_joints)
    # Using world coordinate frame, scoot in y-pos direction
    new_base_pos = np.array(init_base_pos) + np.array([0.0, 0.4, 0.0])
    pb.set_joint_positions(world.robot, world.base_joints, new_base_pos)

    # Move robot forward to get in gripping range
    x_goal = 1.0  # got this from playing around in sim
    x = new_base_pos[0]
    while x > x_goal:
        goal_pos = translate_linearly(world, 0.01)
        pb.set_joint_positions(world.robot, world.base_joints, goal_pos)
        time.sleep(0.02)  # let's make it look smooth
        x = goal_pos[0]

    # Try out sample function
    tool_link = pb.link_from_name(world.robot, "panda_hand")
    sample_fn = get_sample_fn(world.robot, world.arm_joints)
    # conf = sample_fn()
    # print("Conf = ", conf)
    # pb.set_joint_positions(world.robot, world.arm_joints, conf)
    pb.wait_for_user()
    for i in range(5):
        print("Iteration: ", i)

        # Use IK to move
        ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
        print("IK joints = ", ik_joints)
        start_pose = pb.get_link_pose(world.robot, tool_link)
        print("Start pose = ", start_pose)
        # TODO I think these are in robot body pose, FRD maybe based on motion
        end_pose = pb.multiply(start_pose, pb.Pose(pb.Point(x=0.2)))
        print("End pose = ", end_pose)
        for pose in pb.interpolate_poses(start_pose, end_pose, pos_step_size=0.01):
            conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
            if conf is None:
                print("Failure!")
                pb.wait_for_user()
                break
            pb.set_joint_positions(world.robot, ik_joints, conf)

    # TODO set up RobotPlanner object that reads in PDDL files and initializes activity + motion planners

