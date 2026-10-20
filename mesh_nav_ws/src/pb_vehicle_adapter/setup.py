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
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", package_files("launch/*.launch.py")),
        ("share/" + package_name + "/config", package_files("config/*")),
        ("share/" + package_name + "/rviz", package_files("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="nav_test maintainers",
    maintainer_email="maintainers@example.invalid",
    description="MeshNav navigation coordinator and real-robot velocity interface.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "meshnav_navigator = pb_vehicle_adapter.meshnav_navigator:main",
            "pb_cmd_vel_adapter = pb_vehicle_adapter.cmd_vel_adapter:main",
        ],
    },
)
