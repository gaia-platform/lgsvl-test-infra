# Looks for processes with `ros-args` and kills them.
# Useful for getting rid of zombie ROS processes.
ps -eo pid,cmd | grep 'ros-args' | grep -v grep | awk '{print $1}' | xargs kill