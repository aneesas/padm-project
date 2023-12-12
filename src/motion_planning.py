import os
import sys

# Local project code
from src.helpers import Node

# For importing provided simulator code
sys.path.extend(os.path.abspath(os.path.join(os.path.dirname(os.getcwd()),
                                             *["padm_project_2023f", d])) for d in ["", "pddlstream", "ss-pybullet"])

from src.world import World

### Helper functions


### RRT Planner

def sample(world: World):
    # TODO can I get bounds from world?
    return

def in_obstacle(world: World, pose: tuple):
    # TODO
    return False

def nearest_node(V: list, node: Node) -> Node:
    return Node()

def near(pose1: tuple, pose2: tuple, tolerance=0.1):
    # TODO how many elements does pose have? 6?
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

    path = []
    while num_iterations < max_iterations:
        # Sample from free space to get x_rand
        # Try goal-biasing every N samples
        if num_iterations % n_goal_bias == 0:
            # x_rand = rut.sample(end_region.bounds)
            x_rand = goal_pose
        else:
            x_rand = rut.sample(bounds)

        # If x_rand is in an obstacle, ignore
        # TODO
        if in_obstacle(world, x_rand):
            num_iterations += 1
            continue

        # Find nearest node to x_rand in V -> x_nearest
        x_nearest = nearest_node(V, x_rand)

        # Steer from x_nearest to x_rand to get x_new
        # TODO use inverse kinematics to check if feasible or not
        x_new = rut.steer(x_nearest, x_rand)
    
        # If the path from x_nearest to x_new encounters a collision, ignore
        # TODO path collision-checking
        if rut.collision(x_nearest, x_new, radius, environment):
            num_iterations += 1
            continue
        
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