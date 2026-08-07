from setuptools import find_packages, setup

package_name = 'robot_tf2_diagnostics'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot_autonomy',
    maintainer_email='dev@robotautonomy.dev',
    description='TF2 tree validation and diagnostics.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf2_diagnostics = robot_tf2_diagnostics.tf2_diagnostics_node:main',
        ],
    },
)