from enum import Enum
import casadi as ca
import numpy as np

import math

class Waypoint:
    def __init__(self, x: float | None, y: float | None, heading: float | None, resolution: int, guess: bool):
        self.x = x
        self.y = y
        self.theta = heading
        self.resolution = resolution
        self.guess = guess

    def x_or(self, default):
        return self.x if self.x != None else default

    def y_or(self, default):
        return self.y if self.y != None else default

    def theta_or(self, default):
        return self.theta if self.theta != None else default

    def x_or_default(self):
        return self.x if self.x != None else 0.0

    def y_or_default(self):
        return self.y if self.y != None else 0.0

    def theta_or_default(self):
        return self.theta if self.theta != None else 0.0


def rotate_vector_by_expr(x, y, theta):
    return [x * ca.cos(theta) - y * ca.sin(theta),
            x * ca.sin(theta) + y * ca.cos(theta)]


def apply_derivative_constraint(opti, x: ca.MX, xdot: ca.MX, dt: ca.MX):
    for i in range(1, x.size1()):
        opti.subject_to(x[i] == x[i - 1] + (xdot[i] * dt[i]))


def solve_net_torque(theta, f_x, f_y, module_translations):
    def tau(m):
        module = module_translations[m]
        [x_m, y_m] = rotate_vector_by_expr(module[0], module[1], theta)
        return x_m * f_y[m] - y_m * f_x[m]
    return sum(map(tau,
                   range(0, len(module_translations))))

Constraint = Enum('Constraint', ['MAX_TRANS_VEL', 'MAX_ANG_VEL', 'VEL_DIRECTION'])
def generate_swerve_trajectory(
        waypoints: list[Waypoint],
        bumper_translations: list[tuple[float, float]],
        module_translations: list[tuple[float, float]],
        max_vel: float,
        max_torque: float,
        wheel_radius: float,
        moi: float,
        mass: float,
        constraints: list[tuple[int, int, bool, Constraint, tuple[float, ...] | float]]
):
    control_interval_counts = list(map(lambda w: w.resolution, waypoints[:-1]))
    sample_count = sum(control_interval_counts)
    bumper_radius = max(map(lambda t: math.sqrt(
        t[0] * t[0] + t[1] * t[1]), bumper_translations))
    max_wheel_force = max_torque / wheel_radius

    problem = ca.Opti()

    # state variables
    dt = problem.variable(sample_count)
    problem.set_initial(dt, [0.1] * sample_count)

    x = problem.variable(sample_count)
    y = problem.variable(sample_count)
    theta = problem.variable(sample_count)

    vx = problem.variable(sample_count)
    vy = problem.variable(sample_count)
    omega = problem.variable(sample_count)

    ax = problem.variable(sample_count)
    ay = problem.variable(sample_count)
    alpha = problem.variable(sample_count)

    # Module forces
    f_x = problem.variable(sample_count, len(module_translations))
    f_y = problem.variable(sample_count, len(module_translations))

    # minimize total time
    problem.minimize(ca.sum1(dt))
    # dont go back in time
    problem.subject_to(dt[:] >= 10e-10)
    problem.subject_to(dt[:] <= 0.05)

    # kinematics constraints
    apply_derivative_constraint(problem, x, vx, dt)
    apply_derivative_constraint(problem, vx, ax, dt)
    apply_derivative_constraint(problem, y, vy, dt)
    apply_derivative_constraint(problem, vy, ay, dt)
    apply_derivative_constraint(problem, theta, omega, dt)
    apply_derivative_constraint(problem, omega, alpha, dt)

    # force must equal mass time acceleration
    for samp in range(dt.size(1)):
        problem.subject_to(ca.sum2(f_x)[samp] == mass * ax[samp])
        problem.subject_to(ca.sum2(f_y)[samp] == mass * ay[samp])
        problem.subject_to(solve_net_torque(
            theta[samp], f_x[samp, :], f_x[samp, :], module_translations) == moi * alpha[samp])

    # each module must obey speed and force limits
    for samp in range(dt.size1()):
        [vx_prime, vy_prime] = rotate_vector_by_expr(
            vx[samp], vy[samp], -theta[samp])
        vx_m = list(map(lambda m: vx_prime -
                    m[1] * omega[samp], module_translations))
        vy_m = list(map(lambda m: vy_prime +
                    m[0] * omega[samp], module_translations))
        for m in range(0, len(module_translations)):
            problem.subject_to(vx_m[m]**2 + vy_m[m]**2 <= max_vel**2)
            problem.subject_to(f_x[samp, m]**2 +
                               f_y[samp, m]**2 <= max_wheel_force**2)

    # must hit waypoints
    for i in range(len(waypoints)):
        if waypoints[i].guess: continue
        sample = 0 if i == 0 else sum(control_interval_counts[0:i]) - 1
        if waypoints[i].x != None:
            problem.subject_to(x[sample] == waypoints[i].x)
        if waypoints[i].y != None:
            problem.subject_to(y[sample] == waypoints[i].y)
        if waypoints[i].theta != None:
            problem.subject_to(theta[sample] == waypoints[i].theta)
    
    # apply constraints
    for constraint in constraints:
        intervals = []
        # sgmt
        if constraint[2]:
            intervals = range(sum(control_interval_counts[0:constraint[0]]), sum(control_interval_counts[0:constraint[1]]))
        else:
            intervals = [sum(control_interval_counts[0:constraint[0]]), sum(control_interval_counts[0:constraint[1]]) - 1]
        print(intervals)
        for i in intervals:
            if constraint[3] == Constraint.MAX_TRANS_VEL:
                problem.subject_to(vx[i]**2 + vy[i]**2 <= constraint[4]**2)
            elif constraint[3] == Constraint.MAX_ANG_VEL:
                problem.subject_to(omega[i]**2 <= constraint[4]**2)
            elif constraint[3] == Constraint.VEL_DIRECTION:
                problem.subject_to(ca.atan2(vy[i], vx[i]) == constraint)
                

    # set initial guess
    latest_x = 0
    latest_y = 0
    latest_theta = 0
    for sgmt in range(len(waypoints) - 1):
        interval = [sum(control_interval_counts[0: sgmt]), sum(control_interval_counts[0: sgmt + 1])]
        latest_x = waypoints[sgmt].x_or(latest_x)
        latest_y = waypoints[sgmt].y_or(latest_y)
        latest_theta = waypoints[sgmt].theta_or(latest_theta)
        problem.set_initial(
            x[interval[0] : interval[1]], 
            np.linspace(
                latest_x,
                waypoints[sgmt + 1].x_or(latest_x),
                interval[1] - interval[0]))
        problem.set_initial(
            y[interval[0] : interval[1]], 
            np.linspace(
                latest_y,
                waypoints[sgmt + 1].y_or(latest_y),
                interval[1] - interval[0]))
        problem.set_initial(
            theta[interval[0] : interval[1]], 
            np.linspace(
                latest_theta,
                waypoints[sgmt + 1].theta_or(latest_theta),
                interval[1] - interval[0]))

    # debug plot
    timestamps = [0]
    for i in range(1, len(problem.debug.value(dt, problem.initial()))):
        timestamps.append(sum(problem.debug.value(dt, problem.initial())[0:i]))
    initial = {
        "dt": problem.debug.value(dt, problem.initial()),
        "timestamp": timestamps,
        "x":  problem.debug.value(x, problem.initial()),
        "y":  problem.debug.value(y, problem.initial()),
        "vx": problem.debug.value(vx, problem.initial()),
        "vy": problem.debug.value(vy, problem.initial()),
        "ax": problem.debug.value(ax, problem.initial()),
        "ay": problem.debug.value(ay, problem.initial()),
    }

    problem.solver("ipopt")
    solve = problem.solve()
    timestamps = [0]
    for i in range(1, len(solve.value(dt))):
        timestamps.append(sum(solve.value(dt)[0:i]))
    return {
        "dt": solve.value(dt),
        "timestamp": timestamps,
        "x": solve.value(x),
        "y": solve.value(y),
        "theta": solve.value(theta),
        "vx": solve.value(vx),
        "vy": solve.value(vy),
        "omega": solve.value(omega),
        "ax": solve.value(ax),
        "ay": solve.value(ay),
        "alpha": solve.value(alpha),
        "initial": initial
    }