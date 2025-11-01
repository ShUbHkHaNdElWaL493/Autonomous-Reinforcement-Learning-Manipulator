#!/bin/bash
# To be executed from project directory only

colcon build --packages-select description # Build the interfaces required for other packages
source install/setup.bash # Source the project workspace
colcon build # Build the rest of the packages