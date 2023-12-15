import os
import sys
import numpy as np
import time

# Local project code
from motion_planning import rrt
from helpers import add_ycb, move_arm, put_down_sugar, put_down_spam

# Provided simulator code
sys.path.extend(os.path.abspath(os.path.join(os.path.dirname(os.getcwd()),
                                             *["padm_project_2023f", d])) for d in ["", "pddlstream", "ss-pybullet"])

import pybullet_tools.utils as pb

# These are from padm_project_2023f
from src.world import World
from src.utils import COUNTERS, compute_surface_aabb, name_from_type, \
    SUGAR, SPAM, DRAWERS, DRAWER_JOINTS

# Constants
INIT_POSE_SUGAR = (0.05, 0.65, np.pi / 4)  # x, y, yaw in world
INIT_POSE_SPAM = (0.2, 1.1, np.pi / 4)  # x, y, yaw in world
ACTIVITY_PLAN_FILE = "../data/plan.txt"
DRAWER_NAME = "indigo_drawer_top_joint"

# TODO I think we only need the right side
BASE_POSE2D = {
    "left": np.array([0.85, -0.15, np.pi]),
    "right": np.array([0.85, 0.6, np.pi])
}

HAND_POSE3D = {
    "drawer": np.array([0.35, 1.15, -0.57]),
    "counter": np.array([0.7, 1.0, -0.54]),
    "burner": np.array([0.7, 0.0, -0.54])
}

ACTIVITY_GOAL_LOC = {
    "open_drawer": "drawer",
    "close_drawer": "drawer",
    "pick_up_spamatc": "spam",
    "put_down_spamatd": "drawer",
    "pick_up_sugaratb": "sugar",
    "put_down_sugaratc": "counter",
    "movedtoc": "counter",
    "movectod": "drawer",
    "movedtob": "burner",
    "movebtoc": "counter",
    "movebtod": "drawer",
}

add_sugar_box = lambda world, **kwargs: add_ycb(world, SUGAR, **kwargs)
add_spam_box = lambda world, **kwargs: add_ycb(world, SPAM, **kwargs)

def secondary_effects(world, act):
    """Executes actions after motion based on which action name is provided."""
    if "move" in act:
        # Valid action, but nothing else to do
        return

    if act == "open_drawer":
        world.open_door(pb.joint_from_name(world.kitchen, DRAWER_NAME))
        new_x = pb.get_joint_position(world.kitchen, pb.joint_from_name(world.kitchen,
                                                                        DRAWER_NAME))
        HAND_POSE3D["drawer"] = HAND_POSE3D["drawer"] + np.array([new_x, 0., 0.])
    elif act == "close_drawer":
        world.close_door(pb.joint_from_name(world.kitchen, DRAWER_NAME))
        new_x = pb.get_joint_position(world.kitchen, pb.joint_from_name(world.kitchen,
                                                                        DRAWER_NAME))
        HAND_POSE3D["drawer"] = HAND_POSE3D["drawer"] - np.array([new_x, 0., 0.])
    elif act == "pick_up_spamatc":
        # TODO Move spam to link grasp
        return
    elif act == "put_down_spamatd":
        # Move spam to inside drawer and out of link grasp
        spam_pose = put_down_spam()
        HAND_POSE3D["spam"] = spam_pose[0] + np.array([0., 0., 0.2])
        # TODO move spam out of link grasp
    elif act == "pick_up_sugaratb":
        # TODO Move sugar to link grasp
        return
    elif act == "put_down_sugaratc":
        # Move sugar to counter surface and out of link grasp
        sugar_pose = put_down_sugar()
        HAND_POSE3D["sugar"] = sugar_pose[0] + np.array([0., 0., 0.2])
        # TODO move sugar out of link grasp
    else:
        print("[secondary_effects] WARNING: got invalid action ", act)


if __name__ == "__main__":
    print("Random seed:", pb.get_random_seed())
    print("Numpy seed:", pb.get_numpy_seed())

    # world = World(use_gui=True)
    world = World(use_gui=False)

    # Look for obstacles
    print([pb.get_joint_name(world.kitchen, joint) for joint in world.kitchen_joints])

    # Set up simulation world as expected
    # NOTE: Leaving `counter` argument from example out because it has no effect.
    _, pose_sugar = add_sugar_box(world, pose2d=INIT_POSE_SUGAR)
    _, pose_spam = add_spam_box(world, pose2d=INIT_POSE_SPAM)

    # Save object goal locations for later
    HAND_POSE3D["sugar"] = pose_sugar[0] + np.array([0., 0., 0.2])
    HAND_POSE3D["spam"] = pose_spam[0] + np.array([0., 0., 0.2])

    # Move robot to starting position to get in gripping range
    pb.set_joint_positions(world.robot, world.base_joints, BASE_POSE2D["right"])
    pb.wait_for_user()

    # Read in activity plan from file
    with open(ACTIVITY_PLAN_FILE, "r") as fp:
        activity_plan = fp.readlines()
    activity_plan = [line.strip() for line in activity_plan]
    print("Read in activity plan: ", activity_plan)
    pb.wait_for_user()

    # Get path and command arm for each activity
    tool_link = pb.link_from_name(world.robot, "panda_hand")
    for act in activity_plan:
        start_pose = pb.get_link_pose(world.robot, tool_link)
        goal_pose = pb.Pose(point=HAND_POSE3D[ACTIVITY_GOAL_LOC[act]],
                            euler=pb.euler_from_quat(start_pose[1]))
        path = rrt(world, start_pose, goal_pose, tolerance=0.07, d_steer=0.5)
        print("Path received for {} = {}\n".format(act, path))
        move_arm(world, tool_link, path)
        secondary_effects(world, act)
        pb.wait_for_user()

    print("Done executing plan!")
    pb.wait_for_user()

