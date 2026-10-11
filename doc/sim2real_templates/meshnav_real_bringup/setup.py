from glob import glob
from setuptools import setup


package_name = "meshnav_real_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md", "THIRD_PARTY_NOTICES.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="nav_test maintainers",
    maintainer_email="maintainers@example.invalid",
    description="Validated standalone MeshNav real robot bringup.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "meshnav_validate_bundle = meshnav_real_bringup.bundle:main",
        "meshnav_make_smoke_bundle = meshnav_real_bringup.smoke:make_bundle_main",
        "meshnav_fake_hardware = meshnav_real_bringup.smoke:fake_hardware_main",
    ]},
)
