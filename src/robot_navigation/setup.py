from setuptools import find_packages, setup

package_name = 'robot_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
         ['config/nav2_office.yaml',
          'config/nav2_warehouse.yaml',
          'config/nav2_challenging.yaml']),
        ('share/' + package_name + '/behavior_trees',
         ['behavior_trees/mission_tree.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot_autonomy',
    maintainer_email='dev@robotautonomy.dev',
    description='Nav2 configuration profiles for the autonomous robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={},
)