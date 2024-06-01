from enum import Enum
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patch
import matplotlib.animation as anim

import math
import solver as solver
from solver import Waypoint

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

fig = plt.figure()
ax_xy = plt.subplots()[1]

ax_xy.set_title("path xy")
ax_xy.plot(test["x"], test["y"])
ax_xy.scatter(test["x"], test["y"])

dts = test["dt"]
timestamps = [0]
for i in range(1, len(dts)):
    timestamps.append(sum(dts[0:i]))

ax_x = plt.subplots()[1]
ax_x.set_xticks(timestamps)
ax_x.twinx().plot(timestamps, test["x"], color="black")
ax_x.twinx().plot(timestamps, test["vx"], color="red")
ax_x.twinx().plot(timestamps, test["ax"], color="pink")

afig, ax_anim = plt.subplots()
ax_anim.set_title("anim traj")
ax_anim.plot(test["x"], test["y"])
box = patch.Rectangle((test["x"][0] - bumper_trans[1][0], test["y"][0] - bumper_trans[1][1]), bumper_trans[1][0] * 2, bumper_trans[1][1] * 2, angle=test["theta"][0] * 180.0 / math.pi, rotation_point='center', fill=False)
ax_anim.add_patch(box)
framerate = 60.0
def lerp (a, b, t):
    return a + ((b - a) * t)
mod_frame = 0
def update_anim(frame):
    global mod_frame
    mod_frame += 1
    mod_frame = mod_frame % (((test["timestamp"][-1]) * framerate) - 1)
    time = mod_frame/framerate
    s = 0
    s_err = math.inf
    for i, timestamp in enumerate(test["timestamp"]):
        if abs(time - timestamp) <= s_err:
            s_err = abs(time - timestamp)
            s = i
    if test["timestamp"][s] > time:
        s -= 1
    t = (time - test["timestamp"][s]) / (test["timestamp"][s + 1] - test["timestamp"][s])
    # print("time " + '{0:.2f}'.format(time) + "; s stamp " + '{0:.2f}'.format(test["timestamp"][s]) + "; frame " + '{0:.0f}'.format(mod_frame) + "; sample " + str(s) + "; t " + '{0:.2f}'.format(t))
    box.set_xy((lerp(test["x"][s], test["x"][s + 1], t) - bumper_trans[1][0], lerp(test["y"][s], test["y"][s + 1], t) - bumper_trans[1][1]))
    box.set_angle(lerp(test["theta"][s], test["theta"][s + 1], t) * 180.0 / math.pi)
    ax_anim.add_patch(box)
    return

traj_anim = anim.FuncAnimation(fig=afig, 
                         func=update_anim,
                                frames=len(test["x"]), interval=1000/framerate, blit=False)

ax_dt = plt.subplots()[1]
ax_dt.set_title("dts")
ax_dt.plot(test["timestamp"], test["dt"])

plt.show()
