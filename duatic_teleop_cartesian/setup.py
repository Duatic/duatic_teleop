from setuptools import find_packages, setup

package_name = "duatic_teleop_cartesian"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Duatic AG",
    maintainer_email="dev@duatic.com",
    description=(
        "Interactive RViz markers that mirror Cartesian pose topics, publish target poses, "
        "and report pose error."
    ),
    license="BSD-3-Clause",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "interactive_cartesian_node = duatic_teleop_cartesian.interactive_cartesian_node:main",
        ],
    },
)
