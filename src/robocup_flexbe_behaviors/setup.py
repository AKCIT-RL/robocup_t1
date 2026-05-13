#!/usr/bin/env python

import os
from glob import glob
from setuptools import setup

package_name = 'robocup_flexbe_behaviors'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RoboCup Team',
    maintainer_email='robotics@robocup.com',
    description='FlexBE behaviors for RoboCup soccer robot',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robocup_assist_sm = robocup_flexbe_behaviors.robocup_assist_sm',
            'robocup_chase_sm = robocup_flexbe_behaviors.robocup_chase_sm',
            'robocup_game_sm = robocup_flexbe_behaviors.robocup_game_sm',
        ],
    },
)
