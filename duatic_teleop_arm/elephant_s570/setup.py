from setuptools import find_packages, setup

package_name = "elephant_s570"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/visualize.launch.py"]),
        ("share/" + package_name + "/rviz", ["rviz/s570.rviz"]),
        ("share/" + package_name + "/urdf", ["urdf/s570.urdf"]),
        (
            "share/" + package_name + "/urdf/meshes",
            [
                "urdf/meshes/base.dae",
                "urdf/meshes/link1.dae",
                "urdf/meshes/link2.dae",
                "urdf/meshes/link3.dae",
                "urdf/meshes/link4.dae",
                "urdf/meshes/link5.dae",
                "urdf/meshes/link6.dae",
                "urdf/meshes/link7.dae",
                "urdf/meshes/mycontroller.dae",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Duatic AG",
    maintainer_email="dev@duatic.com",
    description="Teleoperation interface for the Elephant Robotics myController S570.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "elephant_s570_node = elephant_s570.node:main",
            "s570_hardware_node = elephant_s570.s570_hardware_node:main",
            "publish_teleop_to_rosbridge_node = elephant_s570.publish_teleop_to_rosbridge_node:main",
        ],
    },
)
