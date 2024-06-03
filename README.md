# Tinyoptlib

Tinyoptlib is a lightweight set of python scripts for swerve trajectory optimization for the FIRST Robotics Competition (FRC).
I started work on this after getting fed up with working on [Trajoptlib](https://github.com/SleipnirGroup/TrajoptLib) because of its excessive verbosity for the task at hand and my own lack of familiarity with C++.
This repo is more than anything meant as a template for anyone else who is looking into swerve trajopt, rather than a standalone tool.
Feel free to reach out with any questions about it.

[solver.py](solver.py) is the meat of this project, and includes the swerve model.

[demo.py](demo.py) is a hacked together set of matplotlib graphs to show the results of running a solve, although its pretty messy at the moment.

[json_gen.py](json_gen.py) is meant to be a PeninsulaPlanner compatible script to read a path json, run it through the solver, and output another json.
It is still a work in progress
