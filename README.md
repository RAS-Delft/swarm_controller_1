# BEP implementing Swarm Intelligence Algorithm for automated vessels

## Overview
This ROS 2 node implements a swarm control algorithm to manage multiple autonomous boats (Tito Neri models). The boats communicate via ROS topics to share their positions, headings, and velocities. The swarm controller receives this information, calculates desired headings and velocities for each boat based on swarm behaviors, and publishes these references to the individual boat controllers.

## Features

* **Swarm Behaviors:**
    * **Cohesion:** Boats are attracted towards the center of mass of their neighbors.
    * **Separation:** Boats avoid collisions with nearby neighbors.
    * **Alignment:** Boats align their velocity (speed and direction) with their neighbors.
    * **Goal Seeking:** Boats navigate towards a goal position clicked in RViz.

* **Tunable Parameters:**
    * Distances for cohesion, separation, and goal seeking.
    * Strengths of the cohesion, separation, alignment, and goal-seeking forces.
    * Maximum speed and minimum speed for heading change.
    * Proportional gains for heading and velocity control.
    * Maximum heading change rate.

## Requirements

* **ROS 2 Humble (or newer):** Ensure you have a compatible version of ROS 2 installed.
* **TurtleBoat Simulation:** You'll need the TurtleBoat repository to simulate the boats. https://github.com/bartboogmans/TurtleBoat.git
* **Visualization:** RViz is used to visualize the boats and set a goal position by clicking.
https://github.com/RAS-Delft/ras_urdf_common
* **RAS_ROS_Core_Control_Modules:** These modules are necessary to control the individual boats. You'll need to have the ROS Core Control Modules for the TurtleBoat or a similar robot model installed and configured.
https://github.com/RAS-Delft/ras_ros_core_control_modules.git

