# Deliver Master: Indoor Restaurant Delivery Robot

## Overview
This project, developed as part of **MIE1077**, proposes an autonomous indoor restaurant delivery robot designed to address labour shortages and improve service quality in Canadian urban restaurants.  
The system integrates natural language understanding, action planning, object detection, navigation, and precise pick-and-place operations within a simulated 3D restaurant environment.

## Problem Statement
Growing restaurant demand in cities like Toronto and Vancouver requires efficient service solutions despite staffing constraints.  
Our robot enhances operational efficiency by autonomously handling routine delivery tasks, ensuring consistent guest experience.

## Key Features
- **Natural Language Understanding** – Interprets guest or staff commands (via typed text in simulation).  
- **Action Planning** – Converts high-level tasks into ordered action sequences.  
- **Object Detection** – Uses YOLOv11m CNN for real-time, robust object localization.  
- **Navigation** – Combines Nav2+SLAM for mapping with DWA for dynamic obstacle avoidance.  
- **Grasping & Placement** – Executes precise pick-and-place motions via imitation learning.

## System Architecture
The software consists of three main modules:
1. **Input Module Package** – GUI for text commands and camera feed; integrates YOLO detection and LLM-based parsing.  
2. **Navigation Module Package** – Global planner with local obstacle avoidance.  
3. **Motion Preparation Package** – Manual tele-operation and autonomous imitation learning modes.

![Architecture](overall_architecture.png)

## Results
- **Simulation Trials** – 50 runs with robust navigation, <16° yaw error, <0.12 m position error.  
- **Object Detection** – F1 Score ~0.99 at confidence threshold 0.72, reliable across wide confidence range.  
- **Imitation Learning** – Behavior Cloning (BC) converged rapidly; Inverse Reinforcement Learning (IRL) yielded effective policies for pick-and-place tasks.

![F1 Score Curve](F1_curve.png)

## Technologies Used
- **Simulation**: Gazebo, SketchUp (environment design)  
- **Frameworks**: ROS2 (Nav2, SLAM), YOLO, LLM (ChatGPT-4.1-nano)  
- **Learning**: Behavior Cloning (BC), Inverse Reinforcement Learning (IRL)

## Assumptions
- Input commands via text (no microphone access in WSL VM).  
- Static 3D restaurant map (simulation only).  
- Fixed initial object placements (sprite cans) with support for relocation.  
- Supports both tele-operation and autonomous execution.

## Future Work
- Transition from simulation to real-world deployment.  
- Extend multi-object manipulation beyond sprite cans.  
- Integrate real voice command input.

## Demo Slides - Video Included
[Deliver Master Demo](https://docs.google.com/presentation/d/1bCcY7AFAmbktMcgvld7WufuGaBtqRXFj/edit?usp=sharing&ouid=114492464970792931615&rtpof=true&sd=true)

---

### Authors
- **Ruiwu Liu**  

*Date: July 25, 2025*
