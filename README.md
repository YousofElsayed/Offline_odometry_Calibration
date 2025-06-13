# Robot Simulation and Calibration

This repository contains code and instructions to run robot simulations in Gazebo, perform sensor calibration, and execute test trajectories using the calibrated values.

## Getting Started

Follow these steps to prepare your workspace, launch the simulation, perform calibration, run experiments, and plot results.

---

## Prepare Workspace

1. Build the ROS workspace:
  `catkin_make`

2. Start the ROS master:
  `roscore`

---

## Launch Gazebo Simulation Environment and Robot

To start the simulation environment with the TurtleBot3 Burger robot, run the following launch file:

  `roslaunch turtle_burger_myfile.launch `

  
To start the simulation environment with the Pioneer 3-DX robot, run the following launch file:

  `roslaunch pioneer_myfile.launch`

---
---

## Starting Calibration Nodes
Run the calibration scripts to obtain calibrated kinematic parameter values:

To use the Triangle calibration Method run:

  `python3 trian_cal.py`
  
To use the Accurate UMBmark Method run:

  `python3 acc_umb.py`
  
To use the Rotational Calibration Method run:

  ` python3 rot_cal.py`
  
---

## Run Test Trajectories with Calibrated Values
After obtanining the calibrated values from the calibration nodes run the node below to drive the test trajectories using the new values.

  `python3 experiment_3.py`

## Plot robot trajectories:

  `python3 plot_experiment_3.py`


---

## Notes

- Ensure you have all necessary ROS dependencies installed and your ROS environment properly sourced.
- Ensure you have all necessary Robot model files installed to run the Gazebo simulation.

## Maintainers
- Yousof Elsayed
 
