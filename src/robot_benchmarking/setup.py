from setuptools import find_packages, setup

package_name = 'robot_benchmarking'

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
    description='Quantitative benchmarking: experiment runner, metrics, graphs, reports.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_experiment = robot_benchmarking.run_experiment:main',
            'generate_report = robot_benchmarking.generate_report:main',
            'generate_gifs = robot_benchmarking.generate_gifs:main',
        ],
    },
)