"""Load PB vehicle simulation parameters."""

import os

from ament_index_python.packages import get_package_share_directory
import yaml


def default_profile_path():
    return os.path.join(
        get_package_share_directory("pb_vehicle_adapter"),
        "config", "pb_vehicle_profile.yaml",
    )


def load_profile(path):
    with open(path, encoding="utf-8") as stream:
        return yaml.safe_load(stream)["pb_vehicle"]["ros__parameters"]
