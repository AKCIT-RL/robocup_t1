from setuptools import setup
from setuptools import find_packages

package_name = 'robocup_flexbe_states'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RoboCup Team',
    maintainer_email='robocup@example.com',
    description='Custom FlexBE states for RoboCup',
    license='Apache 2.0',
    tests_require=['pytest'],
)
