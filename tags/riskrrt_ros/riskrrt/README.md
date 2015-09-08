**installing riskrrt**

$ sudo apt-get install ros-indigo-navigation
$ cd ~/catkin_ws/src
$ git clone https://github.com/Spalanza/riskrrt.git
$ catkin_make



**running the examples scenarios**

some examples scenarios (using stage) are provided in worlds/
run the scenario with
$ roslaunch riskrrt scenario_name.launch
change the planner behavior by modifying the parameters in params/riskrrt_params.yaml



**provided nodes**

controller: a path following node for diffrential drive robots
            in: trajectory (riskrrt/Trajectory) 
                robot pose (geometry_msgs/PoseWithCovarianceStamped)
            out: controller feedback, true if robot is on trajectory (std_msgs/Bool)
                 velocity commands (geometry_msgs/Twist)
goal_pub: automatic goal publisher
          args: pose x (m), pose y (m) with respect to map
          out: goal (geometry_msgs/PoseStamped)
og_builder*: some occupany grid array publishers
             args: number of humans, human speed (m/s)
             in: map (nav_msgs/OccupancyGrid)
                 human odometry (nav_msgs/Odometry)
             out: human velocity (geometry_msgs/Twist)
                  occupancy grid array (riskrrt/OccupancyGridArray)
planner: the riskrrt planner
         in: occupancy grid array (riskrrt/OccupancyGridArray)
             goal (geometry_msgs/PoseStamped)
             controller feedback (std_msgs/Bool)
             robot pose (geometry_msgs/PoseWithCovarianceStamped)
             robot odometry (nav_msgs/Odometry)
         out: trajectory (riskrrt/Trajectory)



**publishing your own map**

use the maxDepth and timeStep parameters from params/riskrrt_params.yaml
-maxDepth should be the occupancy grid array size
-timeStep should be the time gap between each grid
