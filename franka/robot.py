"""Class representing the Franka robot arm"""

from .activity_planner import ActivityPlanner

class Robot():
    """TODO Class docstring"""
    def __init__(self, params):
        self._planner = ActivityPlanner(params["domain_file"], params["problem_file"])
