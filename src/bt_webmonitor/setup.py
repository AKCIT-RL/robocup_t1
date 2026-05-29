from setuptools import setup
import os
from glob import glob

package_name = 'bt_webmonitor'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'static'), glob('bt_webmonitor/static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RoboCup Team',
    maintainer_email='robocup@example.com',
    description='Web-based real-time BehaviorTree monitor',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webmonitor = bt_webmonitor.webmonitor_node:main',
        ],
    },
)
