"""This module provides a TrajectoryOptimizer class for optimizing joint trajectories."""

import numpy as np
from pydrake.all import MathematicalProgram, SnoptSolver, eq


class TrajectoryOptimizer:
    """TrajectoryOptimizer class handles optimization of joint trajectories."""

    def __init__(self, start, goal):
        """
        Initializes TrajectoryOptimizer.

        Args:
        - start (numpy.ndarray): The start joint angles.
        - goal (numpy.ndarray): The goal joint angles.
        """
        self.start = start
        self.goal = goal

        self.total_angle_sum = 0.0
        self.timesteps = 15
        self.duration = 0.05  # seconds

        self.num_joints = 7
        self.joint_angle_limits = np.array([
            [-2.8973, 2.8973],
            [-1.7628, 1.7628, ],
            [-2.8973, 2.8973],
            [-3.0718, -0.0698],
            [-2.8973, 2.8973],
            [-0.0175, 3.7525],
            [-2.8973, 2.8973],
        ])
        self.joint_velocity_limits = np.array([2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100, 2.6100])

    def optimize_trajectory(self):
        """
        Optimizes joint trajectory based on constraints and goals.

        Returns:
        - numpy.ndarray or None: Optimized joint angles if successful, else None.
        """
        prog = MathematicalProgram()

        # Define decision variables for joint angles
        q = prog.NewContinuousVariables(self.num_joints, self.timesteps, "q")
        qdot = prog.NewContinuousVariables(self.num_joints, self.timesteps, "qdot")

        # Add joint limits constraints
        for i in range(self.timesteps):
            for j in range(self.num_joints):
                prog.AddBoundingBoxConstraint(self.joint_angle_limits[j, 0], self.joint_angle_limits[j, 1], q[j, i])

        # Joint velocity limits constraints
        for i in range(self.timesteps):
            for j in range(self.num_joints):
                prog.AddBoundingBoxConstraint(-self.joint_velocity_limits[j], self.joint_velocity_limits[j], qdot[j, i])

        # Add equality constraints to relate joint angles to velocities
        for i in range(self.timesteps):
            if i < self.timesteps - 1:
                prog.AddConstraint(eq(q[:, i + 1], q[:, i] + qdot[:, i] * self.duration))

        distance_to_goal = 0
        for i in range(self.timesteps):
            for j in range(self.num_joints):
                distance_to_goal += (q[j, i] - self.goal[j]) ** 2

        prog.AddCost(distance_to_goal)

        for j in range(7):
            prog.AddLinearConstraint(q[j, 0] == self.start[j])  # Start configuration constraint
            prog.AddLinearConstraint(q[j, self.timesteps - 1] == self.goal[j])  # Goal configuration constraint

        # Solve
        solver = SnoptSolver()
        result = solver.Solve(prog)

        if result.is_success():
            optimized_joint_angles = result.GetSolution(q)
            return optimized_joint_angles
        else:
            print("Optimization failed.")
            return None


# Main function
if __name__ == "__main__":

    # start and goal states (joint angles)
    start = np.array([0.01200158428400755, -0.5697816014289856, 5.6801487517077476e-05, -2.8105969429016113,
                      -0.00025768374325707555, 3.0363450050354004, 0.7410701513290405])
    goal = np.array([-0.20768028027714652, 0.21438383591444632, 0.31978980093429454, -1.0415727534705166,
                     -0.12723213908070363, 2.039780699681913, 0.8158902027978119])

    # optimize!
    trajopt = TrajectoryOptimizer(start, goal)
    output = trajopt.optimize_trajectory()
    print(output)
