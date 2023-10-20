MIT 16.413 final project repo for Aneesa &amp; Anjali

## To do:
- define predicates, actions, and effects (problem file/domain) - Anjali
- ~~choose activity planner~~ graph plan w/FF heuristic
- ~~test out parsing example PDDL files - Aneesa~~
- ~~skeleton of PDDL planner - Aneesa (if time)~~
- set up simulation (need access to sim first)

## Questions:
- 

# High-Level Approach
- Architecture should make it easy to swap out planners
-

# Activity Planning
### Assumptions made
- There is only one drawer in the environment that we are able to manipulate
- Each location (burner, countertop, inside of drawer) can only hold one item at a time
- The robot arm can only hold one item at a time
- We can abstract away the details of certain actions, e.g., how the gripper picks up different items

### Files:
- **activity_planner.py** - contains logic for parsing the PDDL problem and domain files and running the activity planning algorithm
- 

### Key Functions
- TODO

### Approach
- TODO

### Challenges, what did/didn't work, etc.
- TODO

# Motion Planning

# Trajectory Optimization
