"""TODO Module docstring"""

import pddl

class ActivityPlanner:
    """TODO Class docstring"""

    def __init__(self, domain_file, problem_file):
        self._domain = pddl.parse_domain(domain_file)
        self._problem = pddl.parse_problem(problem_file)
        self._problem.domain = self._domain
        # TODO error-checking on input files
        # 1. file DNE
        # 2. file cannot be parsed (bad format, wrong PDDL file type, etc.)

    def solve(self) -> list:
        """Return an activity plan based on input domain/problem files"""
        plan = self._run_ff_planner()
        # can call alternate planners here--can even be passed as parameter if desired
        return plan

    def _run_ff_planner(self) -> list:
        """Implementation of heuristic FF planner"""
        plan = []
        return plan

    # can add alternate planners here
