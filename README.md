Final Project Repo for Aneesa &amp; Anjali  
MIT 16.413 - Fall 2023    

# Planning & Progress
### To do:
- ~~define predicates, actions, and effects (problem file/domain) - Anjali~~
- ~~choose activity planner~~ graph plan w/FF heuristic
- ~~test out parsing example PDDL files - Aneesa~~
- ~~skeleton of PDDL planner - Aneesa (if time)~~
- set up simulation (need access to sim first)

### Questions:
- Simulation?

### High-Level Approach
- Architecture should make it easy to swap out planners

### Timeline
- 10/2 - 10/20: Activity Planning
    - Deliverable Due: 10/21
- 10/23 - 11/17: Motion Planning
    - Deliverable Due: 11/18
- 11/27 - 12/11: Trajectory Optimization
    - Deliverable Due: 12/12??


# Deliverable 1: Activity Planning
### Assumptions
- There is only one drawer in the environment that we are able to manipulate
- Each location (burner, countertop, inside of drawer) can only hold one item at a time
- The robot arm can only hold one item at a time
- We can abstract away the details of certain actions, e.g., how the gripper picks up different items

### Files:
- **activity_planner.py** - contains logic for parsing the PDDL problem and domain files and running the activity planning algorithm
- **pddl_domain.py** - contains PDDL domain definition specifying the types, predicates, and actions
- **pddl_problem.py** - contains PDDL probelm definition specifying the objects, initial state, and goal state

### Key Functions
- **solve** - runs the specified planner (modular design so we can swap out planners)
- **_run_ff_planner**
    - **make_relaxed_graph** - constructs a relaxed plan graph
    - **get_cost** - returns the FF heuristic
    - **best_first_search** - runs Best-First Search
    - **extract_plan** - returns the solution
    - **ff_search** - runs Fast Forward search
    - **extend_graph** - figures out what actions we can take next

### Approach
[Using slides as reference]
- For our planner, we first create a relaxed plan graph and run FF on it (searches backwards for a plan)
- The number of actions in this relaxed plan is used a heuristic to estimate the true cost of achieving the goal
- Our FF heuristic is used to guide which actions to explore when extending the 'real' plan
- When no search progress can be made, we switch to Best First Search

### Challenges
- We had to utilize 'supertypes' in order to generalize our actions to handle different object types in the same way

# Deliverable 2: Motion Planning
### Something
- asdf

# Deliverable 3: Trajectory Optimization
### Something
- asdf