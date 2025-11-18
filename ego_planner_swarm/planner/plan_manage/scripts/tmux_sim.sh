#!/bin/bash

MASTER_URI=http://localhost:11311
SETUP_ROS_STRING="export ROS_MASTER_URI=${MASTER_URI}; source ~/sim_ws/HKUST/ego-planner-swarm/devel/setup.bash"
SESSION_NAME=sim_exp

CURRENT_DISPLAY=${DISPLAY}
if [ -z ${DISPLAY} ];
then
  echo "DISPLAY is not set"
  CURRENT_DISPLAY=:0
fi

if [ -z ${TMUX} ];
then
  TMUX= tmux new-session -s $SESSION_NAME -d
  echo "Starting new session."
else
  echo "Already in tmux, leave it first."
  exit
fi

# Make mouse useful in copy mode
tmux set -g mouse on

tmux rename-window -t $SESSION_NAME "Core"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; roscore" Enter
#tmux split-window -t $SESSION_NAME
tmux select-layout -t $SESSION_NAME tiled

tmux new-window -t $SESSION_NAME -n "Main"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 1; roslaunch mav_simulator tmux_gazebo_sim.launch" Enter # launch world
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 4; roslaunch mav_simulator tmux_spawn.launch" Enter # launch MAV
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 7; roslaunch mav_simulator tmux_control.launch" Enter # launch MAV's controller
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 4; export DISPLAY=${CURRENT_DISPLAY}; rosparam set robot_name ddk1; rosrun rqt_mav_manager rqt_mav_manager" Enter # launch MAV's UI
tmux select-layout -t $SESSION_NAME tiled

tmux new-window -t $SESSION_NAME -n "Exp"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 4; roslaunch ego_planner simple_run.launch" Enter
tmux select-layout -t $SESSION_NAME even-horizontal

# Add window to easily kill all processes
tmux new-window -t $SESSION_NAME -n "Kill"
tmux send-keys -t $SESSION_NAME "tmux kill-session -t ${SESSION_NAME}"

tmux select-window -t $SESSION_NAME:1
tmux -2 attach-session -t $SESSION_NAME

