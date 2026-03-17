from setuptools import find_packages, setup

package_name = 'duatic_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/grasp_positions.yaml']),
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
            'interactive_pyroki_node = duatic_teleop.interactive_pyroki_node:main',
            'pose_sequence_node = duatic_teleop.pose_sequence_node:main',
        ],
    },
)