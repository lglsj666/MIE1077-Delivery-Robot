from setuptools import setup

package_name = 'input_module'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # If YOLO model file is to be installed with package, include it:
        # ('share/' + package_name + '/models', ['input_module/models/yolo11m.pt']),
    ],
    install_requires=['setuptools', 'ultralytics', 'openai', 'opencv-python', 'PyQt5'],  # PyQt5 for GUI
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Input Module for camera handling, YOLO detection, and LLM command parsing',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'input_module_node = input_module.input_module_node:main'
        ],
    },
)
