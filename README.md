MIT 16.413 final project repo for Aneesa &amp; Anjali

## To do:
- define predicates, actions, and effects (problem file/domain) - Anjali
- ~~choose activity planner~~ graph plan w/FF heuristic
- test out parsing example PDDL files
- skeleton of PDDL planner
- set up simulation (need access to sim first)

## Questions:
- Does our PDDL parser support supertyping? Is that a normal thing? (thinking about drawer being both a container and location)
- FF - how to? SOS
- Assumptions?

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
- TODO

### Key Functions
- TODO

### Approach
- TODO

### Challenges, what did/didn't work, etc.
- TODO

# Motion Planning

# Trajectory Optimization
