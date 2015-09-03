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



provided nodes

controller: a path following node for diffrential drive robots
            in: trajectory (riskrrt/Trajectory) 
                robot pose (geometry_msgs/PoseWithCovarianceStamped)
            out: feedback, true if robot is on trajectory (std_msgs/Bool)
                 velocity commands (geometry_msgs/Twist)
goal_pub: automatic goal publisher
          in:
          out:
og_builder*: some occupany grid array publishers
             in:
             out:
planner: the riskrrt planner
         in:
         out:



publishing your own map

use the maxDepth and timeStep parameters from params/riskrrt_params.yaml
-maxDepth should be the occupancy grid array size
-timeStep should be the time gap between each grid
