#!/usr/bin/env python3
"""Print the PB navigation robot SDF without the rendering sensors.

    python3 tools/reduced_robot_model.py models/pb_navigation_robot.sdf

Used by pb_vehicle_sim.launch.py (`spawn_rendering_sensors:=False`) through a
launch Command substitution, so no reduced SDF is checked in.  See
pb_vehicle_adapter/robot_model.py for why the sensors are removed.
"""

import sys
import argparse

from pb_vehicle_adapter.robot_model import simulation_model_xml


def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("model")
    parser.add_argument("--rendering", choices=["True", "False"], default="False")
    parser.add_argument("--diagnostics", choices=["True", "False"], default="False")
    parser.add_argument("--drive", choices=["legacy", "pi"], default="legacy")
    args = parser.parse_args(argv[1:])
    try:
        sys.stdout.write(simulation_model_xml(
            args.model, args.rendering == "True", args.diagnostics == "True", args.drive))
    except (OSError, ValueError) as exc:
        print("reduced_robot_model.py: %s" % exc, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
