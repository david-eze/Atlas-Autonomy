from setuptools import find_packages, setup

package_name = 'robot_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
         ['config/mission.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot_autonomy',
    maintainer_email='dev@robotautonomy.dev',
    description='Mission-level orchestration: recovery manager, semantic navigation, mission action.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'recovery_manager = robot_mission.recovery_manager:main',
            'semantic_navigation = robot_mission.semantic_navigation:main',
            'web_dashboard = robot_mission.web_dashboard:main',
        ],
    },
)