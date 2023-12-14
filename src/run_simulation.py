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

ACTIVITY_LOCATIONS = {
    
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

    print("World now has bodies:")
    print(world.body_from_name)
    pb.wait_for_user()

    print("pybullet pose for sugar box:")
    print(pb.get_pose(world.body_from_name[name_sugar]))

    # Scoot robot over for better starting position
    init_base_pos = pb.get_joint_positions(world.robot, world.base_joints)
    # Using world coordinate frame, scoot in y-pos direction
    new_base_pos = np.array(init_base_pos) + np.array([0.0, 0.4, 0.0])
    pb.set_joint_positions(world.robot, world.base_joints, new_base_pos)

    # Move robot forward to get in gripping range
    x_goal = 0.85  # got this from playing around in sim
    x = new_base_pos[0]
    while x > x_goal:
        goal_pos = translate_linearly(world, 0.015)
        pb.set_joint_positions(world.robot, world.base_joints, goal_pos)
        time.sleep(0.02)  # let's make it look smooth for fun
        x = goal_pos[0]

    # Try to move hand to sugar box
    tool_link = pb.link_from_name(world.robot, "panda_hand")
    print("Starting joint configuration:")
    ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
    conf = pb.get_joint_positions(world.robot, ik_joints)
    print(conf)
    pb.wait_for_user()
    start_pose = pb.get_link_pose(world.robot, tool_link)
    print("Start pose = ", start_pose)
    pose_sugar_world = pb.get_pose(world.get_body(name_sugar))
    goal_pose = pb.Pose(point=pose_sugar_world[0], euler=pb.euler_from_quat(start_pose[1]))
    print("Goal pose = ", goal_pose)
    while conf is not None:
        # TODO I think these are in robot body pose, FRD maybe based on motion
        # end_pose = pb.multiply(start_pose, pb.Pose(pb.Point(x=0.05, z=-0.01)))
        # print("End pose = ", end_pose)
        for i, pose in enumerate(pb.interpolate_poses(start_pose, goal_pose, pos_step_size=0.01)):
            conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
            if i % 10 == 0:
                print("\tNext pose = \n\t", pose)
            if near(pose, goal_pose):
                print("Got to {}.\nClose enough!".format(pose))
                conf = None
                pb.wait_for_user()
                break
            if conf is None:
                print("Failure!")
                pb.wait_for_user()
                break
            pb.set_joint_positions(world.robot, ik_joints, conf)

    # TODO set up RobotPlanner object that reads in PDDL files and initializes activity + motion planners

