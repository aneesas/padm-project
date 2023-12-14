import os
import sys
import numpy as np
import time

# Local project code
from motion_planning import rrt, near

# Provided simulator code, which is not set up to be installed as packages
sys.path.extend(os.path.abspath(os.path.join(os.path.dirname(os.getcwd()),
                                             *["padm_project_2023f", d])) for d in ["", "pddlstream", "ss-pybullet"])

import pybullet_tools.utils as pb
from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

# These are from padm_project_2023f
from src.world import World
from src.utils import COUNTERS, compute_surface_aabb, name_from_type, \
    translate_linearly, SUGAR, SPAM

# Constants
UNIT_POSE2D = (0., 0., 0.)  # x, y, yaw
INIT_POSE_SUGAR = (0.05, 0.65, np.pi / 4)  # x, y, yaw in world
INIT_POSE_SPAM = (0.2, 1.1, np.pi / 4)  # x, y, yaw in world

# TODO I think we only need the right side
BASE_POSE2D = {
    "left": np.array([0.85, -0.15, np.pi]),
    "right": np.array([0.85, 0.6, np.pi])
}
HAND_POSE3D = {
    "drawer_closed": np.array([]),
    "drawer_open": np.array([]),
    "drawer_above": np.array([]),
    "drawer_inside": np.array([]),
    "counter": np.array([]),
    "burner": np.array([])
}
ACTIVITY_GOAL_LOC = {
    "open_drawer": "drawer_open",
    "close_drawer": "drawer_closed",
    "pick_up_spamatc": "spam",
    "put_down_spamatd": "drawer_inside",
    "pick_up_sugaratb": "sugar",
    "put_down_sugaratc": "counter",
    "movedtoc": "counter",
    "movectod": "drawer_above",
    "movedtob": "burner",
    "movebtoc": "counter",
}

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

add_sugar_box = lambda world, **kwargs: add_ycb(world, SUGAR, **kwargs)
add_spam_box = lambda world, **kwargs: add_ycb(world, SPAM, **kwargs)

if __name__ == "__main__":
    print("Random seed:", pb.get_random_seed())
    print("Numpy seed:", pb.get_numpy_seed())

    world = World(use_gui=True)

    # Set up simulation world as expected
    # NOTE: Leaving `counter` argument from example out because it has no effect.
    name_sugar, pose_sugar = add_sugar_box(world, pose2d=INIT_POSE_SUGAR)
    name_spam, pose_spam = add_spam_box(world, pose2d=INIT_POSE_SPAM)
    print("{} pose = {}".format(name_sugar, pose_sugar))
    print("{} pose = {}".format(name_spam, pose_spam))

    # Save object goal locations for later
    HAND_POSE3D["sugar"] = pose_sugar[0] + np.array([0., 0., 0.2])
    HAND_POSE3D["spam"] = pose_spam[0] + np.array([0., 0., 0.2])

    pb.wait_for_user()

    # Move robot to starting position to get in gripping range
    pb.set_joint_positions(world.robot, world.base_joints, BASE_POSE2D["left"])
    pb.wait_for_user()
    pb.set_joint_positions(world.robot, world.base_joints, BASE_POSE2D["right"])
    pb.wait_for_user()

    # TODO set up RobotPlanner object that reads in PDDL files and initializes activity + motion planners

    # Move from start to sugar box using RRT
    tool_link = pb.link_from_name(world.robot, "panda_hand")
    start_pose = pb.get_link_pose(world.robot, tool_link)
    rrt_path = rrt(world, start_pose, HAND_POSE3D["sugar"])
    print("Path received = ")
    print(rrt_path)