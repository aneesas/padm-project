"""
This module implements an ActivityPlanner class that utilizes PDDL_Parser to solve activity planning problems.
"""

import sys
sys.path.append('../padm-project-2023f/pddl-parser/pddl_parser')

from PDDL import PDDL_Parser


class ActivityPlanner:
    """
    ActivityPlanner class is used to solve activity planning problems based on PDDL files.
    """

    def __init__(self, domain_file, problem_file):
        """
        Initialize the ActivityPlanner object with domain and problem files.

        Args:
        - domain_file (str): File path to the PDDL domain file.
        - problem_file (str): File path to the PDDL problem file.
        """
        activity = PDDL_Parser()
        activity.parse_domain(domain_file)
        activity.parse_problem(problem_file)

        self.domain_name = activity.domain_name
        self.requirements = activity.requirements
        self.types = activity.types
        self.objects = activity.objects
        self.actions = activity.actions
        self.predicates = activity.predicates
        self.problem_name = activity.problem_name
        self.initstate = activity.state
        self.positive_goals = activity.positive_goals
        self.negative_goals = activity.negative_goals

        # TODO: Perform error-checking on input files
        # - Check if files exist
        # - Validate if the files can be parsed (bad format, wrong PDDL file type, etc.)

    def solve(self) -> list:
        """
        Return an activity plan based on input domain/problem files.

        Returns:
        - list: Activity plan.
        """
        result = self._run_bfs_planner()
        plan, path = self._extract_activity_plan(result)
        # Alternate planners can be called here or passed as parameters if desired

        return plan

    def _run_bfs_planner(self) -> list:
        """
        Run the breadth-first search planner to find a solution.

        Returns:
        - list: Resulting state.
        """
        queue = [State(set(self.initstate))]
        visited = {self.initstate}  # Start state

        while queue:
            current_node = queue.pop(0)  # State variable
            head_N = current_node.state  # All the predicates in that state

            if self.positive_goals.issubset(head_N) and not (self.negative_goals.intersection(head_N)):
                return current_node
            else:
                extended_paths = self.generate_possible_actions(current_node)
                for action in extended_paths:
                    child = self.apply_action(current_node, action)
                    if child.state not in visited:
                        queue = queue + [child]
                        visited.add(frozenset(child.state))

        return None

    def generate_possible_actions(self, state):
        """
        Generate possible actions based on the current state.

        Args:
        - state: Current state object.

        Returns:
        - list: Possible actions.
        """
        possible_actions = []
        state_preds = state.state

        for action in self.actions:
            pos_preds = action.positive_preconditions
            neg_preds = action.negative_preconditions
            if pos_preds.issubset(state_preds) and not (neg_preds.intersection(state_preds)):
                possible_actions.append(action)
        return possible_actions

    def apply_action(self, state, action):
        """
        Apply an action to the state and generate a new state.

        Args:
        - state: Current state object.
        - action: Action to be applied.

        Returns:
        - State: New state after applying the action.
        """
        new_state = State(set(state.state))

        for effect in action.add_effects:
            new_state.state.add(effect)
        for effect in action.del_effects:
            if effect in new_state.state:
                new_state.state.remove(effect)
        new_state.parent = state
        new_state.action = action

        return new_state

    def _extract_activity_plan(self, state, plan=[], path=[]):
        """
        Extract the activity plan from the final state.

        Args:
        - state: Final state object.
        - plan (list): Plan of actions (default=[]).
        - path (list): Path followed to reach the final state (default=[]).

        Returns:
        - tuple: Activity plan and path.
        """
        if state is None:
            return plan[::-1], path[::-1]
        else:
            path.append(state)
            if state.action is not None:
                plan.append(state.action.name)
            return self._extract_activity_plan(state.parent, plan, path)

    def _run_ff_planner(self) -> list:
        """
        Implementation of heuristic FF planner.
        """
        pass
        # Placeholder for the FF planner implementation

    # Other alternate planners can be added here

class State:
    def __init__(self, state, parent=None, action=None, cost=0):
        """
        Initialize the State object.

        Args:
        - state: Set of predicates representing the state.
        - parent: Parent state (default=None).
        - action: Action taken to reach this state (default=None).
        - cost: Cost associated with the state (default=0).
        """
        self.state = state
        self.parent = parent
        self.action = action
        self.cost = cost

def main():
    domain_filename = "pddl\pddl_domain.pddl"
    problem_filename = "pddl\pddl_problem.pddl"
    act = ActivityPlanner(domain_filename, problem_filename)
    plan = act.solve()

    print(plan)

if __name__ == "__main__":
    main()
