#!/bin/bash

WSPATH="$HOME/mcomp_ws/devel/setup.zsh"

tmux has-session -t stretch_sim 2> /dev/null
if [ $? != 0 ]; then
  # create a new session
  tmux new-session -s stretch_sim -n stretch_sim -d

  # roscore
  tmux send-keys -t stretch_sim 'ros; roscore' C-m
  tmux select-layout tiled

  # gazebo nav
  tmux split-window -h -t stretch_sim
  tmux send-keys -t stretch_sim 'source '$WSPATH C-m 
  tmux send-keys -t stretch_sim 'roslaunch stretch_navigation navigation_gazebo_robotiq.launch'
  tmux select-layout tiled

  # perfect loc
  tmux split-window -h -t stretch_sim
  tmux send-keys -t stretch_sim 'source '$WSPATH C-m 
  tmux send-keys -t stretch_sim 'roslaunch task_handler tf.launch'
  tmux select-layout tiled

  # moveit
  tmux split-window -v -t stretch_sim
  tmux send-keys -t stretch_sim 'source '$WSPATH C-m 
  tmux send-keys -t stretch_sim 'roslaunch stretch_robotiq_moveit_config demo_gazebo.launch'
  tmux select-layout tiled

  # task handler
  tmux split-window -h -t stretch_sim
  tmux send-keys -t stretch_sim 'source '$WSPATH C-m 
  tmux send-keys -t stretch_sim 'rosrun task_handler task_handler_node'
  tmux select-layout tiled

  # perception image capture
  tmux split-window -h -t stretch_sim
  tmux send-keys -t stretch_sim 'source '$WSPATH C-m 
  tmux send-keys -t stretch_sim 'roscd task_handler; cd scripts; ./stretch_image_capture.py'
  tmux select-layout tiled

  # perception detection pub
  tmux split-window -h -t stretch_sim
  tmux send-keys -t stretch_sim 'source '$WSPATH C-m 
  tmux send-keys -t stretch_sim 'roscd task_handler; cd scripts; ./stretch_location_publisher.py'
  tmux select-layout tiled

  # image view
  tmux split-window -h -t stretch_sim
  tmux send-keys -t stretch_sim 'source '$WSPATH C-m 
  tmux send-keys -t stretch_sim 'rosrun rqt_image_view rqt_image_view'
  tmux select-layout tiled

fi

if [ -z "$TMUX" ]; then
  tmux attach -t stretch_sim
else
  tmux switch-client -t stretch_sim
fi
