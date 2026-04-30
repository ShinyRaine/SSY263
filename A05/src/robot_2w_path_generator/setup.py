from setuptools import find_packages, setup

import os

from glob import glob

package_name = 'robot_2w_path_generator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
     py_modules=[
        "robot_2w_path_generator.trajectory_generator_module",
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*_launch.py")),
        (os.path.join("share", package_name, "configs"), glob("configs/*.yaml")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dean',
    maintainer_email='deane@chalmers.se',
    description='Path generator for the robot_2w',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          "path_generator = robot_2w_path_generator.scripts.path_generator_node:main",
          "path_client = robot_2w_path_generator.scripts.path_client:main",
          "path_client_from_cam = robot_2w_path_generator.scripts.path_client_from_cam:main",
        ],
    },
)
