from glob import glob
import os
from setuptools import setup


package_name = "pb_vehicle_adapter"


def package_files(pattern):
    return [path for path in glob(pattern) if os.path.isfile(path)]


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md", "THIRD_PARTY_NOTICES.md", "UPSTREAM_LOCK.md"]),
        ("share/" + package_name + "/launch", package_files("launch/*.launch.py")),
        ("share/" + package_name + "/config", package_files("config/*")),
        ("share/" + package_name + "/models", package_files("models/*")),
        ("share/" + package_name + "/urdf", package_files("urdf/*")),
        ("share/" + package_name + "/rviz", package_files("rviz/*")),
        ("share/" + package_name + "/tools", package_files("tools/*")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="nav_test maintainers",
    maintainer_email="maintainers@example.invalid",
    description="PolarBear 2025 Gazebo vehicle integration adapter.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "pb_cmd_vel_adapter = pb_vehicle_adapter.cmd_vel_adapter:main",
            "pb_ground_truth_adapter = pb_vehicle_adapter.ground_truth_adapter:main",
            "pb_preflight = pb_vehicle_adapter.pb_preflight:main",
            "pb_dddmr_map_publisher = pb_vehicle_adapter.dddmr_map_publisher:main",
            "pb_nav_goal = pb_vehicle_adapter.pb_nav_goal:main",
        ],
    },
)
