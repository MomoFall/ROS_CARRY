from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'carry_my_luggage'


def share(directory, pattern='*'):
    """Install the files of directory matching pattern into the share folder."""
    return (os.path.join('share', package_name, directory),
            glob(os.path.join(directory, pattern)))


def share_tree(directory):
    """Install a whole directory tree (Gazebo models) into the share folder."""
    entries = []
    for root, _, files in os.walk(directory):
        if files:
            entries.append((os.path.join('share', package_name, root),
                            [os.path.join(root, name) for name in files]))
    return entries


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        share('launch', '*.launch.py'),
        share('config', '*.yaml'),
        share('rviz', '*.rviz'),
        share('worlds', '*.world'),
    ] + share_tree('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Elina',
    maintainer_email='elina.astier18@gmail.com',
    description='Carry My Luggage: a TurtleBot3 Burger follows its operator with its LiDAR.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'leg_detector = carry_my_luggage.leg_detector_node:main',
            'operator_tracker = carry_my_luggage.operator_tracker_node:main',
            'gesture_detector = carry_my_luggage.gesture_detector_node:main',
            'follow_manager = carry_my_luggage.follow_manager_node:main',
            'sim_operator = carry_my_luggage.sim_operator_node:main',
        ],
    },
)
