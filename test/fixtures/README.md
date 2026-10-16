# Test fixtures

`okapi/api/units/{RQuantity,QLength,QAngle,QTime}.hpp` are unmodified copies of the
[OkapiLib](https://github.com/purduesigbots/OkapiLib) 5.1.0 unit headers (MPL-2.0, each file keeps its
own license header).

EZ-Template no longer uses OkapiLib. These four files exist only so `compile_matrix.sh` can simulate a user
project that still has okapilib installed, which is the state of nearly every existing PROS project.
Do not edit them, and do not add anything else from OkapiLib here.
