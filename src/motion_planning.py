import os
import sys
import numpy as np

# Local project code
from src.helpers import Node

# For importing provided simulator code
sys.path.extend(os.path.abspath(os.path.join(os.path.dirname(os.getcwd()),
                                             *["padm_project_2023f", d])) for d in ["", "pddlstream", "ss-pybullet"])

from src.world import World
from src.utils import CIRCULAR_LIMITS

import pybullet_tools.utils as pb

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO
import pybullet_tools.ikfast.ikfast as ik

### Helper functions


### RRT Planner

def get_sample_fn(body, joints, custom_limits={}, **kwargs):
    """Generate a sampling function for arm configurations"""
    lower_limits, upper_limits = pb.get_custom_limits(body, joints, custom_limits, circular_limits=CIRCULAR_LIMITS)
    generator = pb.interval_generator(lower_limits, upper_limits, **kwargs)
    def fn():
        return tuple(next(generator))
    return fn

def sample(world: World, sampling_function):
    # TODO I want to operate in world frame
    # get_joint_positions returns in world frame
    return

def in_obstacle(world: World, pose: tuple):
    # TODO
    return False

def distance(node1: Node, node2: Node):
    # Calculate Euclidean distance between two nodes
    pose1 = np.array(node1.pose)
    pose2 = np.array(node2.pose)
    return np.linalg.norm(pose2 - pose1)

def nearest_node(V: list, node: Node) -> Node:
    distances = np.array([distance(x, node) for x in V])
    idx = np.argmin(distances)
    return V[idx]

def steer_panda(world: World, x_from: Node, x_to: Node, d: float=0.5) -> Node:
    # Figure out how far we can go using inverse kinematics
    # Only steer d * max allowed distance (so we don't get stuck in
    # some fully-extended configuration that's hard to get out of)
    tool_link = pb.link_from_name(world.robot, "panda_hand")
    ik_joints = ik.get_ik_joints(world.robot, PANDA_INFO, tool_link)
    # TODO how do I get 
    new_pose = ()
    return Node(new_pose, parent=x_from)

def near(pose1: tuple, pose2: tuple, tolerance=0.1) -> bool:
    """
    Returns true if the poses are within tolerance of each other in
    all (x, y, z) dimensions.

    Can handle either ((x, y, z), (quat)) or (x, y, z) poses
    """
    if len(pose1) == 2:
        pose1 = pose1[0]
    if len(pose2) == 2:
        pose2 = pose2[0]
    if abs(pose1[0] - pose2[0]) <= tolerance and \
        abs(pose1[1] - pose2[1]) <= tolerance and \
        abs(pose1[2] - pose2[2]) <= tolerance:
        return True
    else:
        return False

def trace_path(node: Node) -> list:
    path = [node.pose]
    parent = node.parent
    while parent is not None:
        path.append(parent.pose)
        parent = parent.parent
    path.reverse()
    return path

def rrt(world: World, start_pose, goal_pose, tolerance, max_iterations=1e10, n_goal_bias=10):
    """
    max_iterations (int): limit planning time with number of iterations
    n_goal_bias (int): sample from goal region every n samples

    Returns list of poses for collision-free path from start to goal
    """
    # Start tree (V) with x_init only
    x_init = Node(start_pose, parent=None)
    V = [x_init]

    # Limit planning time with # of iterations
    num_iterations = 0

    # Create sampling function
    sampling_function = get_sample_fn(world.robot, world.arm_joints)

    path = []
    while num_iterations < max_iterations:
        # Sample from free space to get x_rand
        # Try goal-biasing every N samples
        if num_iterations % n_goal_bias == 0:
            # x_rand = rut.sample(end_region.bounds)
            x_rand = goal_pose
        else:
            x_rand = sample(world, sampling_function=sampling_function)

        # If x_rand is in an obstacle, ignore
        # TODO
        if in_obstacle(world, x_rand):
            num_iterations += 1
            continue

        # Find nearest node to x_rand in V -> x_nearest
        x_nearest = nearest_node(V, x_rand)

        # Steer from x_nearest to x_rand to get x_new
        x_new = steer_panda(world, x_nearest, x_rand, d=0.5)
    
        # If the path from x_nearest to x_new encounters a collision, ignore
        # TODO path collision-checking
        # if rut.collision(x_nearest, x_new, radius, environment):
        #     num_iterations += 1
        #     continue
        
        # Add x_new to tree
        V.append(x_new)

        # If x_new is in goal region, trace back nodes to x_init to get path
        if near(x_new, goal_pose, tolerance):
            path = trace_path(x_new)
            break

        # Continue searching
        num_iterations += 1

    if len(path) == 0:
        raise Exception("No solution found!")

    return path