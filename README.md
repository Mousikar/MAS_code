# MAS_code
some code about Multi-Agent Systems

仓库中的每个文件夹是一个 工作空间 或者是 Windows的文件夹

rostopic pub -r 10 /robot_1/cmd_vel geometry_msgs/Twist "linear:  x: -0.50
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
rostopic pub -r 10 /robot_1/cmd_vel geometry_msgs/Twist '{linear: {x: -0.2, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.0}}'
rostopic pub -r 10 /robot_2/cmd_vel geometry_msgs/Twist '{linear: {x: -0.2, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.0}}'
rostopic pub -r 10 /robot_3/cmd_vel geometry_msgs/Twist '{linear: {x: -0.2, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.0}}'

