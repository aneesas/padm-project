Final Project Repo for Aneesa &amp; Anjali  
MIT 16.413 - Fall 2023

# Introduction 

This project comprises three main components: Activity Planning, Motion Planning, and Trajectory Optimization, each tackling distinct challenges in the realm of robot manipulation within a simulated kitchen environment.

## Learning Objectives
The project was structured to achieve the following objectives:

- Designing PDDL domain and problem files for task planning.
- Implementing an activity planner using various techniques studied in the course.
- Developing sample-based motion planning algorithms for the robot's manipulator.
- Utilizing trajectory optimization techniques to refine motion plans for smoother execution.

# Deliverable 1: Activity Planning

This module implements an ActivityPlanner class that serves as a solver for activity planning problems utilizing the Planning Domain Definition Language (PDDL).

In this section, our primary task was to first create a domain and problem file using PDDL for a kitchen environment. The specified objectives included removing a sugar box from the burner and stowing a spam box inside a drawer. We then developed an activity planner to produce a proposed plan for execution.

## Assumptions in Domain Design

We made the following assumptions in modeling the problem:
- There is only one drawer in the environment that we are interested in manipulating.
- Each location (burner, countertop, inside of drawer) can only hold one item at a time.
- The robot arm can only hold one item at a time.
- We can abstract away the details of certain actions, e.g., how the gripper picks up different items.

## Files and Key Functions

### Files:
- `src/activity_planner.py`: Implements the `ActivityPlanner` class that serves as the core of the activity planning functionality.
- `pddl/pddl_domain.pddl` - Contains the PDDL domain definition including specification of the types, predicates, and actions
- `pddl/pddl_problem.pddl` - Contains the PDDL problem definition including specification of the objects, initial state, and goal state
- `padm-project-2023f/pddl-parser/pddl-parser/PDDL.py`: From the pddl-parser library. Contains the PDDL parsing functionalities used to read domain and problem files. Note: We have placed this folder in .gitignore, so the repo would need to be included in order for this code to run properly.

### Key Classes and Functions:

#### ActivityPlanner Class:
- `__init__(self, domain_file, problem_file)`: Initializes the ActivityPlanner object by parsing domain and problem files using the PDDL_Parser class from the pddl-parser library.
- `solve(self)`: Generates an activity plan based on the input domain and problem files.
- `_run_bfs_planner(self)`: Implements a breadth-first search planner to find a solution.
    - `generate_possible_actions(self, state)`: Generates possible actions based on the current state.
    - `apply_action(self, state, action)`: Applies an action to the state and generates a new state.
- `_extract_activity_plan(self, state, plan, path)`: Extracts the activity plan from the final state.

#### State Class:
- `__init__(self, state, parent, action, cost)`: Initializes the State object representing a state in the planning process.

## Approach Used for Plan Generation

The approach for plan generation involves:
- Parsing the PDDL domain and problem files using the `PDDL_Parser`.
- Initializing an `ActivityPlanner` object with the parsed information.
- Utilizing a breadth-first search planner (`_run_bfs_planner`) to find a solution by exploring possible actions and states.
- Extracting the resulting activity plan through the `_extract_activity_plan` function.

## Challenges Faced and Strategies Employed

During the development of the activity planning module, challenges included:
- Designing effective predicates, actions, and types to accurately model the domain.
    - We tried to utilize 'supertypes' in order to generalize our actions to handle different object types in the same way, but we ran into issues. After testing with multiple different pddl parsing libraries, we determined that none of them would efficiently be able to handle 'supertypes'. Thus, we had to switch back to the original library and rewrite our predicates and actions so as to be more explicit and allow for more straightforward parsing.
    - Replacement/Population of terms in order to quickly check which actions were feasible from a given state proved difficult. We came across many errors and ultimately decided to go with a more robust implementation that removed the need for parameters. This greatly improved our workflow.
- Ensuring correct parsing of PDDL files and handling errors in case of incorrect file formats or missing information.
- Implementing our initial planning approach using the FF heuristic proved to be quite complex, so we modified our approach to follow Breadth-First Search instead.


Strategies employed to address these challenges involved:
- Iterative refinement of predicates and actions based on domain understanding.
- Extensive testing and validation of the PDDL parsing functionalities.
- Experimenting with different planning algorithms and heuristics to optimize the solution process. The structure of the code allows for ease in switching between different planners if we were to implement other kinds as well.

## Example Plan for Specified Task
**Initial State:**
- 'clear_gripper'
- 'countertop_has_spambox'
- 'at_drawer'
- 'burner_has_sugarbox'
- 'clear_drawer'

**Goal State:**
- 'countertop_has_sugarbox'
- 'drawer_has_spambox'
- not 'open_drawer'

**Generated Plan:**
1. open_drawer (Open the drawer)
2. movedtoc (Move from drawer to countertop)
3. pick_up_spamatc (Pick up spam box from countertop)
4. movectod (Move from countertop to drawer)
5. put_down_spamatd (Put down spam box in the drawer)
6. close_drawer (Close the drawer)
7. movedtob (Move from drawer to burner)
8. pick_up_sugaratb (Pick up sugar box from burner)
9. movebtoc (Move from burner to countertop)
10. put_down_sugaratc (Put down sugar box at countertop)


# Deliverable 2: Motion Planning
The second section involved sample-based motion planning for the robot manipulator within the simulated environment. This required designing a planner for the manipulator and integrating it with the activity planner.

## Assumptions in Design
- The robot doesn't need to determine where exactly to place the items once picked up; that is, the location of the countertop and the drawer are known.
- We are not accounting for the gripper in motion planning (required angle of approach for grasping, positioning of the individual fingers, etc.)

## Key Files and Functions
The relevant code lives in the `motion_planning.py` module. Key functions include:
- `sample(bounds)`: Generates a random 3D point within specified bounds.
- `in_obstacle(world, node, obstacles)`: Checks if a given node position is within any obstacles in the world.
- `distance(node1, node2)`: Calculates the Euclidean distance between two nodes.
- `nearest_node(V, node)`: Finds the closest node in the given tree `V` based on Euclidean distance.
- `steer_panda(world, x_from, x_to, obstacles, d)`: Generates a new node in the direction of `x_to` from `x_from`, constrained by robot arm kinematics and distance factor `d`.
- `near(pose1, pose2, tolerance)`: Checks if two poses are within a specified tolerance in (x, y, z) dimensions.
- `trace_path(node)`: Returns the path from the tree root to the current node assuming correct parent assignments.
- `rrt(world, start_pose, goal_pose, tolerance, max_iterations, n_goal_bias, d_steer)`: Implements the algorithm described below with a time-limiting factor of `max_iterations` to prevent endless churning.

## Approach
We use a straightforward RRT implementation for the motion planner. We assume we are given an initial pose (position/orientation pair) and a goal position (3-D, Cartesian), and that we know the bounds of the world and the locations of obstacles within it.

The algorithm steps are:

0. Start a tree with the initial pose as the root node.  
    - For each step in the activity plan, we take the current end effector pose as the initial pose.

(while a complete path is not found, do:)
1. Sample a 3-D point from free space within some specified bounds.  
    - We use a cube defined by (x, y, z) bounds approximated from the sim environment.  
    - We also added goal-biasing: every `N` samples (default 10), choose the goal pose instead of sampling from free space.
2. Check if the sampled point is within an obstacle, and discard it if so.  
    - We defiine our obstacles as the set of AABB shapes/limits from the sim kitchen object. If a generated 3-D point is within any of these bounding boxes, it is within an obstacle.
3. Find the node in the tree that is closest to the new sampled point.  
    - We use simple Euclidean distance for this, as defined in the `near` function.
4. "Steer" from that closest node to the new sampled point.
    - Implemented in `steer_panda`, we use the supplied inverse kinematics determine how far along the new path the end effector is able to move. We limit the total distance by a fractional factor so that any one steering action does not result in the arm being fully extended/contracted in a position that is difficult to maneuver out of.
5. Check if this path to the sampled point intersects an obstacle, and discard the new point if so.  
    - We use the same obstacles as before, for each interpolated pose along the steering path.
6. Add the new point to the tree with the closest-point node as its parent.
7. If the new point is "close enough" to the goal position, retrace its path back to the root and return this complete path. Else, repeat from Step 1.  
    - Input parameter `tolerance` determines what is "close enough" to the goal.

If implementing this for a real-world test, we might want to augment this motion planner to use RRT* instead of RRT, to bring our resulting paths closer to optimality without sacrificing the sparsity benefits of RRT.

## Challenges
- The vast majority of our effort for this task was in working with the actual simulation, rather than in implementing our motion planner. The sim is large, complex, and difficult to navigate through function signatures alone. This was, overall, both challenging and frustrating. We elaborate on some particular challenges below that extend beyond this simulation environment.

**Positioning and obstacle avoidance:** 
- The Franka arm has many separate components with many degrees of freedom (arm joints + gripper joints). The positions returned by pybullet for the gripper tool are, to our understanding, centered positions, meaning the actual shape/extent of the gripper should be taken into account when avoiding obstacles. 
- However, since we were not working with manipulating the gripper itself in this project, it was difficult to determine end goal positions for the motion planner that would put the gripper in a believable spot to perform the subsequent action in the activity plan.
    - E.g., is the gripper close enough to the spam box to actually pick it up, without having the fingers going right through the box?) 
- In an actual implementation, there would be a lot more bookkeeping (and planning!) necessary to maneuver the complete arm to perform the tasks at hand.

**Frames:** 
- We have the ability to compute arm joint angle configurations from (position, orientation) poses using the provided inverse kinematics solution; however, we found ourselves wanting the forward kinematics as well to compute what the pose of the end effector _would_ be for a particular possible angle configuration.
- Not all positions returned when querying the simulator are given in the world coordinate frame. Having these different reference frames and coordinates is useful and powerful, but requires thorough bookkeeping and a deeper knowledge of the ins and outs of the simulation and robot dynamics than we had. For this project, we resolved the issue by purely working with the gripper in Cartesian space and letting the provided IK tools compute the corresponding valid joint configurations, but in an actual application, we would want to implement tools to switch freely between reference frames and coordinates.

**Computation, tuning, and robustness:**
- The randomness of RRT means convergence to a valid path can take a highly variable amount of time. This can be fine for offline planning, but we planned each motion right before execution in order to account for any variation in starting positions between subsequent actions (due to the `tolerance` parameter of the `rrt` function). In real-world execution, we would want fast online planning that can adapt/react to changes or uncertainty in the environment. The video below shows the lag between actions from the motion planner running. Possible bottlenecks in the algorithm:
    - Computing inverse kinematics for each set of interpolated poses (we tuned to 0.04 as a step size for interpolation)
    - Checking each interpolated pose along each candidate path for collisions

- The speed and success of RRT is sensitive to the input parameters, particularly because the desired start/end positions were all quite close to each other in space. These include:
    - Steering distance `d`--need to move far enough along each sampled trajectory without getting stuck in odd positions or creating a very spread-out tree
    - Tolerance for reaching goal--as before, how close is close enough?
    - Step size for interpolated poses in steering function--because the overall distances as small, this needed to be tuned to be small enough to provide valid motion while also not being so small as to wildly slow down execution
    - Goal-biasing frequency


## Video
Video below is one full plan (generated with the activity planner) executed with RRT-based motion-planning. Note that the video is at 1.5x speed (for video size/length), and the watermark is from the tool used to speed up the screen recording to 1.5x. The long delays from running RRT are obvious in between the executed steps. Note also that the objects aren't actually _grasped_ by the gripper--we never actually figured out how to do this, and since it wasn't part of the intended activity or motion plan, we simply moved the objects after the arm completed the relevant step in the plan.

https://github.com/aneesas/padm-project/assets/9471211/033943f6-df3f-406b-a1c4-0d04d4259055



# Deliverable 3: Trajectory Optimization

The final section focused on trajectory optimization to refine the motion plans generated by sample-based planners. Using pyDrake, we aimed to optimize a free space manipulator trajectory.

## Files and Key Functions
The primary file is `trajectory_optimizer.py`, containing the `TrajectoryOptimizer` class. Key functions include:
- `__init__`: Initializes the optimizer with start and goal joint angles.
- `optimize_trajectory`: Performs joint trajectory optimization based on constraints and goals. Constraints are added to a Mathematical Program (via pydrake). The cost we are trying to minimize is the squared distance from the current joint angle configuration to the goal state joint angles.

## Solver Used
The code utilizes the `SnoptSolver` from PyDrake for solving the optimization problem.

## Optimization Problem
The optimization problem aims to find an optimal joint trajectory that navigates from a given start configuration to a specified goal configuration while respecting joint angle limits and velocity constraints.


### Formalization
![Problem](trajopt_problemformalization.png)


## Challenges Faced
During the implementation, several challenges were encountered, such as:
- Balancing constraints and the objective function for effective convergence.
- Tuning the duration and the number of timesteps for optimal trajectory generation.
- Handling convergence failures and adjusting solver parameters.

## Example Plan
Here is an example of the robot executing the optimized trajectory for the action 'Move from Burner to Drawer'

![Robot Execution Video](movebtod_video.gif)


## Result Comparison
The resulting optimized trajectory can be compared to the initial sample-based motion plan above (see earlier video, first motion performed by the robot). The optimized trajectory is much smoother than the original one, but it covers far more distance in Euclidean space. We believe this is due to our formulation of the optimization problem in joint configuration space vs. running our motion planner in Euclidean space. The trajectory optimizer is limited to continuous motion along the individual joint angles towards the goal joint angles, whereas the motion planner computes "nearness" as a 3-D distance to the goal position.

# Conclusion
## Reflection & Discussion
This project highlighted the importance of planner design decisions in developing an autonomous system that can 1. achieve its intended goals, 2. in a reasonable amount of time, and 3. without generating too much inefficiency. We see the effects of our design choices in:
- Including movement of the arm from position to position as an action in the activity plan; on reflection, we could have left the `move_to_` actions out and let the motion planner handle that aspect. We would have likely had faster resulting execution had we done this.
- Planning in Euclidean vs. joint configuration space--as described above, we can see the difference in what the optimized trajectory looks like vs. our sample-based motion plan.
- Offline vs. online planning--we generated the full activity plan offline, but did the motion planning online, and the consequences are evident in the lag between actions in the video above.

Unfortunately, most of the truly time-consuming challenges we encountered involved working with the pybullet simulation. While powerful and extensive, it has an extremely steep learning curve, and focusing on this aspect of the project was not as illuminating as far as how to design/implement effective planners.

## Individual Contributions

The team collectively deliberated on all aspects, and in terms of execution, Anjali focused on Parts 1 (Activity Planning) & 3 (Trajectory Optimization), and Aneesa implemented Part 2 (Motion Planning) and the simulation interaction.

## References
 - Course lectures
 - Underactuated Robotics relevant chapters








