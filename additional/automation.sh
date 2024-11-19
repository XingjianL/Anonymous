# light-temp <color temperature, light intensity, exposure>
# disease-filter <list of disabled diseases from the sim>
# split-height-leaf <list of (height, number of leaf) pairs>
# preprocess <PlantVillage Processing>

# cold temperature strobe light
RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,40,-3 --both --seed 510 --disease-filter 0,2,3,4,6,7,8,9 --split-height-leaf 75,16,125,8 --preprocess Lab

# without strobe light
RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 0,0,1 --both --seed 510 --disease-filter 0,2,3,4,6,7,8,9 --split-height-leaf 75,16,125,8 --preprocess Lab
