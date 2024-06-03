import solver as solver
from solver import Waypoint
import json
import sys

path = json.load(open(sys.argv[1]))
config = path["config"]
side_length = config["side_length"]
gear_ratio = 5.36 / 1.0
wheel_radius = 0.0508
print(path)
generated = solver.generate_swerve_trajectory(
    list(map(lambda w: Waypoint(w["x"], w["y"], w["theta"], 40, False), path["nodes"])),
    [(-side_length, side_length), (side_length, side_length), (side_length, -side_length), (-side_length, -side_length)],
    # this shouldnt be the same, right?
    [(-side_length, side_length), (side_length, side_length), (side_length, -side_length), (-side_length, -side_length)],
    ((5800 * config["free_speed_percent"]) / (60 * gear_ratio)) if config["FOC"] else ((6000 * config["free_speed_percent"]) / (60 * gear_ratio)),
    ((9.37 * config["efficiency_percent"]) * gear_ratio) if config["FOC"] else ((7.09 * config["efficiency_percent"]) * gear_ratio),
    wheel_radius,
    config["moment_of_inertia"],
    config["mass"],
    []
)
print(generated)
generated.pop("initial")
json.dump({
    "state": list(map(
        lambda s:
            {
                "timestamp": generated["timestamp"][s],
                "x": generated["x"][s],
                "vx": generated["vx"][s],
                "ax": generated["ax"][s],
                "y": generated["y"][s],
                "vy": generated["vy"][s],
                "ay": generated["ay"][s],
                "theta": generated["theta"][s],
                "omega": generated["omega"][s],
                "alpha": generated["alpha"][s]
            },
            range(len(generated["timestamp"]))
    ))
}, open(sys.argv[2], 'w+'))
