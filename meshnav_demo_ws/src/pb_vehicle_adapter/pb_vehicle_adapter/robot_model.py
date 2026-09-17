"""Robot-model helpers for the PB simulation launch.

Gazebo Fortress renders `gpu_lidar` and `camera` sensors even in server-only
mode, so on a machine without a GPU (software EGL) those three sensors drop the
simulation's real-time factor from ~1.0 to ~0.06 and make any closed-loop run
unusable.  None of the three navigation frameworks use them: MeshNav, JIE and
DDDMR navigate from their own static maps and only need the ground-truth TF and
/odom.  `pb_vehicle_sim.launch.py` therefore spawns this reduced model when
`spawn_rendering_sensors:=False`, so the model keeps one source of truth.
"""

import xml.etree.ElementTree as ET


RENDERING_SENSORS = (
    "front_rplidar_a2",         # gpu_lidar
    "front_mid360_lidar",       # gpu_lidar
    "front_industrial_camera",  # camera
)


def strip_rendering_sensors(root):
    """Remove the rendering sensors from an SDF root in place; return the names."""
    removed = []
    for parent in root.iter():
        for sensor in list(parent):
            if sensor.tag == "sensor" and sensor.get("name") in RENDERING_SENSORS:
                parent.remove(sensor)
                removed.append(sensor.get("name"))
    return removed


def reduced_model_root(path):
    """Parse `path` and return (root, removed names) without the rendering sensors."""
    tree = ET.parse(path)
    root = tree.getroot()
    return root, strip_rendering_sensors(root)


def reduced_model_xml(path):
    """Return the robot SDF as a string without the rendering sensors."""
    root, removed = reduced_model_root(path)
    if sorted(removed) != sorted(RENDERING_SENSORS):
        raise ValueError("expected %s, removed %s"
                         % (sorted(RENDERING_SENSORS), sorted(removed)))
    return ET.tostring(root, encoding="unicode")
