from setuptools import setup

package_name = 'motion_preparation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/motion_preparation.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Keyboard teleop, GUI and recorder for TIAGo motion data collection',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_node = motion_preparation.teleop_node:main',
            'recorder_node = motion_preparation.recorder_node:main',
            'gui_node = motion_preparation.gui_node:main',
            'grasp_attacher_node = motion_preparation.grasp_attacher_node:main',   # 新增行
        ],
    },
)
