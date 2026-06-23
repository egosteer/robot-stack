# Sourced by the container's ~/.bashrc (wired up by create_container.sh).
# Edit freely — this file is mounted into the container, so changes take effect in the next
# shell without rebuilding the image or recreating the container.

source /opt/ros/humble/setup.bash

alias cb="colcon build"
alias s="source install/setup.bash"

# robot --teleop | --inference | --replay : build, source, and bring up the hardware for that mode
#   --teleop     cameras + dexterous hands (absolute glove mapping) + data gloves
#   --inference  cameras + dexterous hands (model-driven) + arms
#   --replay     dexterous hands (model-driven) + arms, no cameras (actuators for replay)
robot() {
  colcon build && source install/setup.bash || return 1
  case "$1" in
    --teleop)    ros2 launch robot_bringup robot.launch.py mode:=teleop ;;
    --inference) ros2 launch robot_bringup robot.launch.py mode:=inference ;;
    --replay)    ros2 launch robot_bringup robot.launch.py mode:=replay ;;
    *) echo "usage: robot --teleop | --inference | --replay" >&2; return 1 ;;
  esac
}

alias interface="cb && s && ros2 launch model_interface model_interface.launch.py"
alias collect="cb && s && ros2 launch teleop_collection teleop_collection.launch.py"
alias replay="cb && s && ros2 launch replay_rrd replay_rrd.launch.py"
alias mock="cb && s && ros2 launch mock_robot_data mock_data.launch.py"
alias steamvr="~/workspace/robot-stack/assets/SteamVR/launch.sh"
