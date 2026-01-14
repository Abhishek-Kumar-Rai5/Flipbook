# Interactive MATLAB Flipbook Animations

## Overview
This project is an interactive implementation of the **flipbook animation concept** using MATLAB.  
A flipbook works by displaying a sequence of frames with small graphical changes, creating the illusion of motion.  
Building on this idea, this project uses **MATLAB App Designer (mlapp)** to create GUI-based animations where users can control motion through commands.

Instead of static animations, the system allows **user-driven animation control**, making the flipbook concept interactive and extensible.

---

## Key Features
- Flipbook-style frame-by-frame animations
- Interactive GUI built using MATLAB App Designer
- Command-based animation control
- Real-time graphical rendering using MATLAB axes
- Modular design supporting multiple independent animations
- Extensive use of geometric and linear transformations

---

## Implemented Animations

### 1. Crane Animation
- Rotate, extend, and displace crane arm
- Lift, swing, tilt, and place objects
- Sequential command execution
- Visual tracking of placed objects

### 2. Rocket Animation
- Command-driven rocket motion
- Controlled directional movement and animation flow

### 3. Maze Solver
- Visual navigation through a maze
- Step-by-step path progression animation

### 4. Linear Algebra Transformer Game
- Visual demonstration of transformations such as:
  - Translation
  - Rotation
  - Scaling
- Designed to help understand linear algebra concepts visually

### 5. Sound-Responsive Flipbook
- Animation behavior reacts dynamically to sound input
- Demonstrates basic signal interaction with graphics

---

## How the System Works
- Each animation is constructed as a sequence of frames rendered on a MATLAB UIAxes component.
- User commands are collected via the GUI and stored in a command list.
- Each command modifies animation state variables (position, rotation, scale, etc.).
- Transformations are applied using matrix operations.
- Animations are executed sequentially to simulate smooth motion.

---

## Technologies Used
- MATLAB
- MATLAB App Designer (mlapp)
- 2D Graphics and Animation
- Linear Algebra (matrix transformations)
- Event-driven programming

---

## How to Run the Project
1. Open MATLAB (R2020b or later recommended).
2. Clone this repository:
   ```bash
   git clone https://github.com/your-username/interactive-matlab-flipbook.git
