# Import necessary libraries
import numpy as np
import pydrake
# pydrake imports
from pydrake.all import (
    Variable,
    SymbolicVectorSystem,
    DiagramBuilder,
    LogVectorOutput,
    Simulator,
    Integrator,
    ConstantVectorSource,
    MathematicalProgram,
    Solve,
    SnoptSolver,
    PiecewisePolynomial,
    MeshcatVisualizer,
    ModelVisualizer,
    Parser,
    LinearQuadraticRegulator,
    AddMultibodyPlantSceneGraph,
    Linearize,
    IpoptSolver, 
    StartMeshcat,
    eq, le, ge
)


class TrajectoryOptimizer:

    def __init__(self, start, goal, urdf_path):
        self.start = start
        self.goal = goal
        self.urdf_path = urdf_path

        self.total_angle_sum = 0.0
        self.timesteps = 100 #seconds
        self.duration = 0.5 #seconds
        
        # self.joint_angle_limits = np.array([
        #                         [min_angle_1, max_angle_1],
        #                         [min_angle_2, max_angle_2],
        #                         [min_angle_2, max_angle_2],
        #                         [min_angle_2, max_angle_2],
        #                         [min_angle_2, max_angle_2],
        #                         [min_angle_2, max_angle_2],
        #                         [min_angle_2, max_angle_2],
        #                         ])

    def optimize_trajectory(self):
        # Load the URDF file using MultibodyPlant
        builder = DiagramBuilder()
        plant, scene_graph = pydrake.multibody.plant.AddMultibodyPlantSceneGraph(builder, time_step=0.0)
        Parser(plant).AddModelFromFile(self.urdf_path)
        plant.Finalize()

        # Get joint limits from MultibodyPlant
        joint_limits = []
        for joint_index in range(plant.num_joints()):
            joint = plant.get_joint(joint_index)
            if joint.is_floating():
                continue  # Skipping floating joints
            joint_limits.append([joint.position_lower_limits(), joint.position_upper_limits()])

        # Assuming you have time samples
        # time_steps = np.linspace(0, 1, self.timesteps)  # Example time samples

        # Initialize MathematicalProgram
        prog = MathematicalProgram()

        # Define decision variables for joint angles
        q = prog.NewContinuousVariables(plant.num_positions(), self.timesteps, "q")

        # Add joint limits constraints
        for i in range(self.timesteps):
            for j in range(plant.num_positions()):
                if not plant.get_joint(j).is_floating():
                    joint_index = plant.get_joint(j).position_start()
                    lower_limit, upper_limit = joint_limits[joint_index]
                    prog.AddBoundingBoxConstraint(lower_limit, upper_limit, q[j, i])

        # Define the objective function: minimize the total sum of joint angles
        total_joint_angles = sum(sum(q[:, i]) for i in range(self.timesteps))
        prog.AddCost(total_joint_angles)

        # Solve the optimization problem
        solver = SnoptSolver()
        result = solver.Solve(prog)

        if result.is_success():
            optimized_joint_angles = result.GetSolution(q)
            return optimized_joint_angles
            # Process the optimized joint angles
        else:
            print("Optimization failed.")











        # # Define time parameters
        # time_steps = np.linspace(0, duration, num_time_samples)

        # # Initialize MathematicalProgram
        # prog = MathematicalProgram()

        # # Define decision variables for joint positions, velocities, and accelerations
        # num_positions = 7  # Assuming 7 joints for Franka arm
        # q = prog.NewContinuousVariables(num_positions, num_time_samples, "q")
        # v = prog.NewContinuousVariables(num_positions, num_time_samples, "v")
        # a = prog.NewContinuousVariables(num_positions, num_time_samples - 1, "a")

        # # Add cost function: minimize squared joint accelerations
        # for i in range(num_time_samples - 1):
        #     prog.AddCost(np.sum(a[:, i]**2))

        # # Add dynamics constraint
        # for i in range(num_time_samples - 1):
        #     # Assuming some dynamics function dynamics_constraints(q, v, a) is defined
        #     dynamics_constraint = dynamics_constraints(q[:, i], v[:, i], a[:, i])
        #     for constraint in dynamics_constraint:
        #         prog.AddConstraint(constraint)

        # # Add joint limits constraint
        # # Assuming joint_limits_constraints(q, v, a) is defined
        # joint_limits_constraint = joint_limits_constraints(q, v, a)
        # for constraint in joint_limits_constraint:
        #     prog.AddConstraint(constraint)

        # # Add waypoint constraints: start and end configurations
        # # Assuming q_start and q_goal are defined
        # prog.AddConstraint(q[:, 0] == q_start)
        # prog.AddConstraint(q[:, -1] == q_goal)

        # # Solve the optimization problem
        # solver = SnoptSolver()
        # result = Solve(prog, solver)

        # if result.is_success():
        #     # Extract optimized trajectory
        #     optimized_q = result.GetSolution(q)
        #     # Visualize the optimized trajectory using simulation tools in pyDrake
        #     # (visualization code depends on the simulation environment and setup)
        # else:
        #     print("Optimization failed!")


















# Main function
if __name__ == "__main__":
    # Assuming initial and final joint configurations are given
    urdf_path = "/panda_arm.urdf"
    start = np.array([0.01200158428400755, -0.5697816014289856, 5.6801487517077476e-05, -2.8105969429016113, -0.00025768374325707555, 3.0363450050354004, 0.7410701513290405])
    goal = np.array([-0.20768028027714652, 0.21438383591444632, 0.31978980093429454, -1.0415727534705166, -0.12723213908070363, 2.039780699681913, 0.8158902027978119])

    # Call the optimization function
    trajopt = TrajectoryOptimizer(start, goal, urdf_path)
    output = trajopt.optimize_trajectory()
    print(output)
