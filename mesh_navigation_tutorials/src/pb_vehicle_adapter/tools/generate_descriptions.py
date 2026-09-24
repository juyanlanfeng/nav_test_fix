#!/usr/bin/env python3
"""Expand the PB XMacro wrapper and generate an RViz URDF from the same SDF."""

import argparse
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


def _set_sensor_topic(root: ET.Element, sensor_name: str, topic: str) -> None:
    sensor = root.find(".//sensor[@name='%s']" % sensor_name)
    if sensor is None:
        raise RuntimeError("expanded PB model does not contain sensor '%s'" % sensor_name)
    existing = sensor.find("topic")
    if existing is None:
        existing = ET.SubElement(sensor, "topic")
    existing.text = topic


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xmacro", required=True, type=Path)
    parser.add_argument("--robot-description-root", required=True, type=Path)
    parser.add_argument("--resources-root", required=True, type=Path)
    parser.add_argument("--output-sdf", required=True, type=Path)
    parser.add_argument("--output-urdf", required=True, type=Path)
    arguments = parser.parse_args()

    resource_path = ":".join(
        [
            str(arguments.robot_description_root / "resource" / "models"),
            str(arguments.resources_root / "resource" / "models"),
        ]
    )
    os.environ["IGN_GAZEBO_RESOURCE_PATH"] = resource_path
    # sdformat_tools reads the resource search path at import time.
    from sdformat_tools.urdf_generator import UrdfGenerator
    from xmacro.xmacro4sdf import XMLMacro4sdf

    xmacro = XMLMacro4sdf()
    xmacro.set_xml_file(str(arguments.xmacro))
    xmacro.generate()
    root = ET.fromstring(xmacro.to_string())
    _set_sensor_topic(root, "chassis_imu", "/robot/imu")
    _set_sensor_topic(root, "front_rplidar_a2", "/robot/scan")
    _set_sensor_topic(root, "front_mid360_lidar", "/robot/cloud/points")
    _set_sensor_topic(root, "front_industrial_camera", "/robot/camera/image")

    arguments.output_sdf.parent.mkdir(parents=True, exist_ok=True)
    arguments.output_sdf.write_text(
        "<?xml version=\"1.0\"?>\n" + ET.tostring(root, encoding="unicode") + "\n",
        encoding="utf-8",
    )
    urdf = UrdfGenerator()
    urdf.parse_from_sdf_string(arguments.output_sdf.read_text(encoding="utf-8"))
    arguments.output_urdf.parent.mkdir(parents=True, exist_ok=True)
    arguments.output_urdf.write_text(urdf.to_string(), encoding="utf-8")


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("PB description generation failed: %s" % error, file=sys.stderr)
        raise
