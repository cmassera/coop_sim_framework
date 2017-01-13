# Simulation Framework for Cooperative Vehicles

This repository contains a ROS-based vehicle dynamics and communication simulator for cooperative (interconnected) vehicles. Both longitudinal dynamics and lateral are available, however no actuator dynamics is modeled.

This simulator was developed as part of the paper "Safe Optimization of Highway Traffic with Robust Model Predictive Control-based Cooperative Adaptive Cruise Control”. Some additional results from this paper’s proposed controller are also present in this repository.

## Running the simulator

The simulator requires:
- ROS (with catkin)
- Python 2.7

To execute, add the simulator folder to your ROS path and call any of the available launch files. Vehicle parameters, communication delays and others can be defined in the launch file.

## Citing

@article{massera2016safely,
  title={Safely Optimizing Highway Traffic-A Study on Robust Model Predictive Control for Cooperative Adaptive Cruise Control},
  author={Massera, Carlos M and Terra, Marco H and Wolf, Denis F},
  journal={arXiv preprint arXiv:1605.07493},
  year={2016}
}