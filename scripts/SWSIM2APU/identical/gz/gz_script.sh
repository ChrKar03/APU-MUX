# 1. Set graphics mode
export QT_QPA_PLATFORM=xcb

# 2. Set paths
export GAZEBO_MODEL_PATH=~/Desktop/HITL/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=~/Desktop/HITL/ardupilot_gazebo/build:$GAZEBO_PLUGIN_PATH

# 3. Launch the full simulator
gazebo --verbose ~/Desktop/HITL/ardupilot_gazebo/worlds/iris_arducopter_runway.world
