from setuptools import find_packages, setup

package_name = 'robot_exploration'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
         ['config/exploration.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot_autonomy',
    maintainer_email='dev@robotautonomy.dev',
    description='Frontier-based autonomous exploration for the robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'exploration_node = robot_exploration.exploration_node:main',
        ],
    },
)