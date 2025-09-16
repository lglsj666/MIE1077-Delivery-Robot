# File: input_module/input_module/__init__.py
"""
input_module

This package provides:
- A ROS 2 node that subscribes to Tiago's RGB/depth camera topics
- Real-time YOLO-based object detection (for "sprite")
- GUI interface (PyQt5) to display camera feed and accept natural language commands
- Snapshot functionality invoked via ROS topic or GUI 'test' mode
- Automatic parsing of user commands into action sequences via OpenAI GPT‑4.1‑nano
- Feedback to Grab Module via standard ROS messages / topics

Contains:
- InputModuleNode class (defined in input_module_node.py), instantiated via console script
"""

# Optional: expose the main InputModuleNode class for easy import
from .input_module_node import InputModuleNode

# Define the public API
__all__ = ['InputModuleNode']
