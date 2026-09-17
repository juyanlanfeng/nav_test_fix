#!/usr/bin/env python3
"""Print the PB navigation robot SDF without the rendering sensors.

    python3 tools/reduced_robot_model.py models/pb_navigation_robot.sdf

Used by pb_vehicle_sim.launch.py (`spawn_rendering_sensors:=False`) through a
launch Command substitution, so no reduced SDF is checked in.  See
pb_vehicle_adapter/robot_model.py for why the sensors are removed.
"""

import sys

from pb_vehicle_adapter.robot_model import reduced_model_xml


def main(argv):
    if len(argv) != 2:
        print("usage: reduced_robot_model.py <robot.sdf>", file=sys.stderr)
        return 2
    try:
        sys.stdout.write(reduced_model_xml(argv[1]))
    except (OSError, ValueError) as exc:
        print("reduced_robot_model.py: %s" % exc, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
