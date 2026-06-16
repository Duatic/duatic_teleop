import os
from setuptools import find_packages, setup

package_name = "duatic_teleop_gamepad"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name, "config"), ["config/gamepad_config.yaml"]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Timo Schwarzer",
    maintainer_email="tschwarzer@duatic.com",
    description="Gamepad-based teleoperation interface for Duatic robots.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": ["gamepad_interface = duatic_teleop_gamepad.main:main"],
    },
)
