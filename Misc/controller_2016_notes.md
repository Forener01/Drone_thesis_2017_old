# Controller
- takes inputs from pose_estimation (pose) and path_planning (poseref)
- publishes in cmd_vel and navdata
- function reguXY
  - inputs: x_mes/y_mes, x_desired/y_desired
  - outputs: xvel_cmd/yvel_cmd
- function controlLoop
  - runs regulators from Pose and Poseref
  - publishes the results into cmd_vel

# Path planning
- takes inputs from pose_estimation (pose) and strategy
- publishes in path_planning (poseref) and mapcell
- function xy_desired
  - performs a labyrinth trajectory
- function advanced_xy_desired
  - similar to xy_desired
- function SetRef
  - loads new positions into "this" pointer
- main
  
