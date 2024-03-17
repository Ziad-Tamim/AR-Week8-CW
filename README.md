# AR-Week8-CW
code create by Ziad Tamim
230973161
date: 17/03/2024

## Description:

The `ar_week8_test` package is designed to compute and visualize cubic polynomial trajectories in ROS (Robot Operating System). It includes nodes for generating random trajectory parameters, computing trajectory coefficients based on these parameters, and plotting the resulting trajectories.


## Package Contents:

- **Nodes**:
  - `points_generator.py`: Generates random trajectory parameters and publishes them.
  - `cubic_traj_planner.py`: Subscribes to trajectory parameters, computes coefficients, and publishes them.
  - `compute_cubic_coeffs.py`: Provides a service to compute trajectory coefficients.
  - `plot_cubic_traj.py`: Subscribes to trajectory coefficients and publishes trajectory plots.
- **Services**:
  - `compute_cubic_traj.srv`: Computes cubic trajectory coefficients given trajectory parameters.
- **messages**:
  - 'cubic_traj_coeffs.msg'
  - 'cubic_traj_params.msg'  
- **Launch Files**:
  - `cubic_traj_gen.launch`: Launches all nodes and starts the rqt_plot GUI for visualization.


## To run the package:

1) unzip the ar_week8_test.zip folder in your catkin workspace src folder:
2) build your catkin workspace
3) run the cubic_traj_gen.launch launch file.
