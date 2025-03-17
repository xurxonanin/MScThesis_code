#!/bin/bash

# Clean previous executions
if [ -d build ]; then
  rm -rf build
fi
if [ -d install ]; then
  rm -rf install
fi
if [ -d log ]; then
  rm -rf log
fi

# # Activate the virtual environment
# if [ -f .venv/bin/activate ]; then
#   # source .venv/bin/activate
# else
#   echo "Error: .venv/bin/activate not found."
#   exit 1
# fi

pip install -r requirements.txt

# Source ROS 2 Humble setup
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
else
  echo "Error: /opt/ros/humble/setup.bash not found."
  exit 1
fi

# Source additional ROS 2 setup
if [ -f ~/ros2_humble/install/local_setup.bash ]; then
  source ~/ros2_humble/install/local_setup.bash
else
  echo "Error: ~/ros2_humble/install/local_setup.bash not found."
  exit 1
fi

# Check if colcon is available
if ! command -v colcon &> /dev/null; then
  echo "Error: colcon could not be found."
  exit 1
fi

# Build the workspace
colcon build --packages-select turtlebot3_multi_robot

# Source the workspace setup again to ensure the environment is updated
source ./install/setup.bash

# Ensure ROS 2 binaries are in the PATH again
export PATH=$PATH:/opt/ros/humble/bin

# Ensure the virtual environment's site-packages directory is in PYTHONPATH again
export PYTHONPATH=$PYTHONPATH:$(python -c "import site; print(site.getsitepackages()[0])")

# Check if ros2 is available
if ! command -v ros2 &> /dev/null; then
  echo "Error: ros2 could not be found."
  exit 1
fi

# Launch the combined launch file
ros2 launch turtlebot3_multi_robot gazebo_multi_nav2_world.launch.py