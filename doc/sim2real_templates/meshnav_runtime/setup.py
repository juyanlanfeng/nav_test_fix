from setuptools import setup

setup(
    name="meshnav_runtime",
    version="0.1.0",
    packages=["meshnav_runtime"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/meshnav_runtime"]),
        ("share/meshnav_runtime", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="nav_test maintainers",
    maintainer_email="maintainers@example.invalid",
    description="MeshNav coordinator and explicit-enable command gateway.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "meshnav_navigator = meshnav_runtime.meshnav_navigator:main",
        "command_gateway = meshnav_runtime.command_gateway:main",
    ]},
)
