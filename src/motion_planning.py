import os
import sys
import numpy as np

# Local project code
from helpers import Node

# For importing provided simulator code
sys.path.extend(os.path.abspath(os.path.join(os.path.dirname(os.getcwd()),
                                             *["padm_project_2023f", d])) for d in ["", "pddlstream", "ss-pybullet"])

from src.world import World
from src.utils import ALL_SURFACES, compute_surface_aabb

import pybullet_tools.utils as pb

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO
import pybullet_tools.ikfast.ikfast as ik

### Helper functions

WORLD_BOUNDS = ((-2.0, 2.0), (-2.0, 2.0), (-3.0, 3.0))  # x, y, z--approximated from sim
ALL_KITCHEN_LINKS = ['extractor_hood', 'range', 'front_left_stove', 'front_right_stove', 
                     'back_left_stove', 'back_right_stove', 'control_panel', 'back_left_knob', 
                     'front_left_knob', 'back_right_knob', 'front_right_knob', 'baker_anchor_link', 
                     'baker_link_tmp', 'baker_link', 'baker_handle',
                     'chewie_door_right_anchor_link', 'chewie_door_right_link_tmp', 
                     'chewie_door_right_link', 'chewie_door_right_handle', 
                     'chewie_door_left_anchor_link', 'chewie_door_left_link_tmp', 
                     'chewie_door_left_link', 'chewie_door_left_handle', 'dagger', 
                     'dagger_door_left_anchor_link', 'dagger_door_left_link_tmp', 
                     'dagger_door_left_link', 'dagger_door_left_handle', 
                     'dagger_door_right_anchor_link', 'dagger_door_right_link_tmp', 
                     'dagger_door_right_link', 'dagger_door_right_handle', 'hitman_tmp', 
                     'hitman_countertop', 'hitman', 'hitman_drawer_top', 'hitman_drawer_top_front',
                     'hitman_drawer_handle_top', 'hitman_drawer_bottom', 
                     'hitman_drawer_bottom_front', 'hitman_drawer_handle_bottom', 'indigo_tmp', 
                     'indigo_countertop', 'indigo', 'indigo_door_right_anchor_link', 
                     'indigo_door_right_joint_anchor_link', 'indigo_door_right_link', 
                     'indigo_door_right', 'indigo_door_right_nob_link',
                     'indigo_door_left_anchor_link', 'indigo_door_left_joint_anchor_link', 
                     'indigo_door_left_link', 'indigo_door_left', 'indigo_door_left_nob_link', 
                     'indigo_drawer_top', 'indigo_drawer_handle_top', 'indigo_drawer_bottom', 
                     'indigo_drawer_handle_bottom']


def sample(bounds=WORLD_BOUNDS) -> Node:
    """ Generates random 3D valued (Cartesian) Node from given bounds """
    assert len(bounds) == 3
    min_x, max_x = bounds[0]
    min_y, max_y = bounds[1]
    min_z, max_z = bounds[2]
    rng = np.random.default_rng()
    x = min_x + rng.random() * (max_x - min_x)
    y = min_y + rng.random() * (max_y - min_y)
    z = min_z + rng.random() * (max_z - min_z)
    return Node(pb.Pose(point=np.array([x, y, z])))


def in_obstacle(world: World, node: Node, obstacles=None) -> bool:
    """ Checks if the position of the given node is within any obstacles in world """
    point_3d = node.pose[0]
    if obstacles is None:
        obstacles = [compute_surface_aabb(world, name) for name in ALL_SURFACES]
    for o in obstacles:
        if pb.aabb_contains_point(point_3d, o):
            return True
    return False


def distance(node1: Node, node2: Node):
    """ Calculates Euclidean distance between two nodes """
    return np.linalg.norm(node2.pose[0] - node1.pose[0])


def nearest_node(V: list, node: Node) -> Node:
    """ Finds the closest node in the given tree V based on Euclidean distance """
    distances = np.array([distance(x, node) for x in V])
    idx = np.argmin(distances)
    return V[idx]


def steer_panda(world: World, x_from: Node, x_to: Node, obstacles: list, d: float=0.5) -> Node:
    """
    Generates a new node in the direction of x_to from x_from, limiting 
    the distance from x_from by the robot arm's kinematics and by a scaling
    distance factor d
    """
    # Figure out how far we can go using inverse kinematics
    # Only steer d * max allowed distance (so we don't get stuck in
    # some fully-extended configuration that's hard to get out of)
    tool_link = pb.link_from_name(world.robot, "panda_hand")
    # Let's assume our samples are (position, orientation)
    end_pose = pb.Pose(point=x_to.pose[0], euler=pb.euler_from_quat(x_from.pose[1]))
    interpolated_poses = pb.interpolate_poses(x_from.pose, end_pose, pos_step_size=0.04)
    valid_poses = []
    for i, p in enumerate(interpolated_poses):
        conf = next(ik.closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, p, max_time=0.05), None)
        if conf is None:
            break
        else:
            valid_poses.append(p)
    
    # i represents how far we got into interpolated_poses
    i = int(i * d)
    # Collision detection on path
    print("[steer_panda] got {} valid poses".format(len(valid_poses)))
    for pose in valid_poses[:i+1]:
        if in_obstacle(world, Node(pose), obstacles=obstacles):
            print("No good!")
            return None
    new_pose = valid_poses[i]
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
    """
    Returns the path from tree root to the current node, assuming all
    parent nodes have been correctly assigned
    """
    path = [node.pose]
    parent = node.parent
    while parent is not None:
        path.append(parent.pose)
        parent = parent.parent
    path.reverse()
    return path


### RRT Planner
def rrt(world: World, start_pose: tuple, goal_pose: tuple, tolerance=0.1, 
        max_iterations=1e10, n_goal_bias=10, d_steer=0.5):
    """
    max_iterations (int): limit planning time with number of iterations
    n_goal_bias (int): sample from goal region every n samples
    d_steer (float): distance factor for use in steering function

    Returns list of poses for collision-free path from start to goal
    """
    # Start tree (V) with x_init only
    x_init = Node(start_pose, parent=None)
    V = [x_init]

    # Limit planning time with # of iterations
    num_iterations = 0

    # Get obstacles from world
    obstacles = [pb.get_aabb(world.kitchen, pb.link_from_name(world.kitchen, name)) 
                 for name in ALL_KITCHEN_LINKS]

    path = []
    while num_iterations < max_iterations:
        # Sample from free space to get x_rand
        # Try goal-biasing every N samples
        if num_iterations % n_goal_bias == 0:
            x_rand = Node(goal_pose)
        else:
            x_rand = sample()

        # If x_rand is in an obstacle, ignore
        if in_obstacle(world, x_rand, obstacles=obstacles):
            num_iterations += 1
            continue

        # Find nearest node to x_rand in V -> x_nearest
        x_nearest = nearest_node(V, x_rand)

        # Steer from x_nearest to x_rand to get x_new
        x_new = steer_panda(world, x_nearest, x_rand, obstacles, d=d_steer)
        if x_new is None:
            # This means there was a collision in the path
            num_iterations += 1
            continue

        # Add x_new to tree
        V.append(x_new)

        # If x_new is in goal region, trace back nodes to x_init to get path
        if near(x_new.pose, goal_pose, tolerance):
            path = trace_path(x_new)
            break

        # Continue searching
        num_iterations += 1

    if len(path) == 0:
        if (num_iterations >= max_iterations):
            print("[rrt] Exceeded {} iterations in search".format(max_iterations))
        raise Exception("No solution found!")

    return path