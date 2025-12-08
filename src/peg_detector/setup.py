from setuptools import setup, find_packages

package_name = 'peg_detector'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cobot',
    maintainer_email='cobot@todo.todo',
    description='Shape-based peg detector using RealSense RGB-D camera',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'peg_detector_node = peg_detector.peg_detector_node:main',
        ],
    },
)
