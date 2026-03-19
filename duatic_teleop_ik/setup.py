from setuptools import find_packages, setup

package_name = 'duatic_teleop_ik'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Duatic AG',
    maintainer_email='dev@duatic.com',
    description='High-performance, JIT-compiled inverse kinematics solver using PyRoki and JAX.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interactive_pyroki_node = duatic_teleop_ik.interactive_pyroki_node:main',
        ],
    },
)
