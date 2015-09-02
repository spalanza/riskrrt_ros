installing riskrrt

$ sudo apt-get install ros-indigo-navigation
$ cd ~/catkin_ws/src
$ git clone https://github.com/Spalanza/riskrrt.git
$ catkin_make

running the examples scenarios

some examples scenarios (using stage) are provided in worlds/
run the scenario with
$ roslaunch riskrrt scenario_name.launch
change the planner behavior by modifying the parameters in params/riskrrt_params.yaml

publishing your own map


