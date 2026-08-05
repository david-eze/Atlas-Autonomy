from setuptools import find_packages, setup

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/simulation.launch.py',
          'launch/mapping.launch.py',
          'launch/navigation.launch.py',
          'launch/exploration.launch.py',
          'launch/mission.launch.py']),
        ('share/' + package_name + '/rviz',
         ['rviz/robot_dashboard.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot_autonomy',
    maintainer_email='dev@robotautonomy.dev',
    description='Launch files for the autonomous robot simulation stack.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={},
)
