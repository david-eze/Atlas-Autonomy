from setuptools import find_packages, setup

package_name = 'robot_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
         ['config/slam_toolbox.yaml', 'config/map_quality.yaml']),
        ('share/' + package_name + '/maps',
         ['maps/.gitkeep']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot_autonomy',
    maintainer_email='dev@robotautonomy.dev',
    description='SLAM configuration and map management for the autonomous robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_quality_monitor = robot_mapping.map_quality_node:main',
        ],
    },
)