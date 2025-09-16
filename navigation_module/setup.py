from setuptools import setup

package_name = 'navigation_module'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # 为 ament 索引注册资源
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # 安装 package.xml
        ('share/' + package_name, ['package.xml']),
        # 安装 launch 文件
        ('share/' + package_name + '/launch', ['launch/navigation.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RUIWU LIU',
    maintainer_email='rev.liu@mail.utoronto.ca',
    description='ROS 2 navigation module integrating Nav2, obstacle avoidance, and feedback for TIAGo',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # 注册 navigation_node 可执行脚本
            'navigation_node = navigation_module.navigation_node:main',
        ],
    },
)
