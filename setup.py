from setuptools import setup

package_name = 'waypoint_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name + '/launch', ['launch/waypoint_manager.launch.py']),
    ],
    install_requires=['setuptools', 'PyQt6'],
    python_requires='>=3.8',
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Manage 2D waypoints from RViz and publish nav_msgs/Path',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'waypoints_gui = waypoint_manager.waypoints_gui:main',
    ]},
)
