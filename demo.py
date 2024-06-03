from enum import Enum
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patch
import matplotlib.animation as anim

import math
import solver as solver
from solver import Waypoint

def show_debug_plots(generated, bumper_trans):
    fig = plt.figure()
    ax_xy = plt.subplots()[1]

    ax_xy.set_title("path xy")
    ax_xy.plot(generated["x"], generated["y"])
    ax_xy.scatter(generated["x"], generated["y"])

    dts = generated["dt"]
    timestamps = [0]
    for i in range(1, len(dts)):
        timestamps.append(sum(dts[0:i]))

    ax_x = plt.subplots()[1]
    ax_x.set_xticks(timestamps)
    ax_x.twinx().plot(timestamps, generated["x"], color="black")
    ax_x.twinx().plot(timestamps, generated["vx"], color="red")
    ax_x.twinx().plot(timestamps, generated["ax"], color="pink")

    afig, ax_anim = plt.subplots()
    ax_anim.set_title("anim traj")
    ax_anim.plot(generated["x"], generated["y"])
    box = patch.Rectangle((generated["x"][0] - bumper_trans[1][0], generated["y"][0] - bumper_trans[1][1]), bumper_trans[1][0] * 2, bumper_trans[1][1] * 2, angle=generated["theta"][0] * 180.0 / math.pi, rotation_point='center', fill=False)
    ax_anim.add_patch(box)
    framerate = 60.0
    def lerp (a, b, t):
        return a + ((b - a) * t)
    mod_frame = 0
    def update_anim(frame):
        nonlocal mod_frame
        mod_frame += 1
        mod_frame = mod_frame % (((generated["timestamp"][-1]) * framerate) - 1)
        time = mod_frame/framerate
        s = 0
        s_err = math.inf
        for i, timestamp in enumerate(generated["timestamp"]):
            if abs(time - timestamp) <= s_err:
                s_err = abs(time - timestamp)
                s = i
        if generated["timestamp"][s] > time:
            s -= 1
        t = (time - generated["timestamp"][s]) / (generated["timestamp"][s + 1] - generated["timestamp"][s])
        # print("time " + '{0:.2f}'.format(time) + "; s stamp " + '{0:.2f}'.format(test["timestamp"][s]) + "; frame " + '{0:.0f}'.format(mod_frame) + "; sample " + str(s) + "; t " + '{0:.2f}'.format(t))
        box.set_xy((lerp(generated["x"][s], generated["x"][s + 1], t) - bumper_trans[1][0], lerp(generated["y"][s], generated["y"][s + 1], t) - bumper_trans[1][1]))
        box.set_angle(lerp(generated["theta"][s], generated["theta"][s + 1], t) * 180.0 / math.pi)
        ax_anim.add_patch(box)
        return

    traj_anim = anim.FuncAnimation(fig=afig, 
                            func=update_anim,
                                    frames=len(generated["x"]), interval=1000/framerate, blit=False)

    ax_dt = plt.subplots()[1]
    ax_dt.set_title("dts")
    ax_dt.plot(generated["timestamp"], generated["dt"])

    plt.show()

def demo():
    # test
    bumper_trans = [(-0.5, 0.5), (0.5, 0.5), (0.5, -0.5), (-0.5, -0.5)]
    test = solver.generate_swerve_trajectory(
        [Waypoint(1.0, 0.0, 0.0, 10, False), Waypoint(0.75, 0.2, None, 20, False), Waypoint(1.0, 1.0, 3.14, 20, True), 
        Waypoint(None, 1.5, 3.14, 20, False), Waypoint(2.0, 1.0, 0.0, 20, False)
        ],
        bumper_trans,
        [(-0.4, 0.4), (0.4, 0.4), (0.4, -0.4), (-0.4, -0.4)],
        20.0,
        5.0,
        0.05,
        4.0,
        50.0,
        [
            # (0, 2, False, Constraint.MAX_TRANS_VEL, 0.0), (0, 2, True, Constraint.MAX_TRANS_VEL, 0.25), (1, 1, True, Constraint.VEL_DIRECTION, -3.14)
            ]
    )

    print(test)
    show_debug_plots(test, bumper_trans)
