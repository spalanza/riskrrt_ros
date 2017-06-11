**installing riskrrt**

$ sudo apt-get install ros-indigo-navigation
$ cd ~/catkin_ws/src
$ git clone https://github.com/Spalanza/riskrrt_ros.git
$ catkin_make



**running the examples scenarios**

some examples scenarios (using stage) are provided in worlds/
run goal node setting eg. x = 1, y = 10 with
$ rosrun riskrrt goal_pub 1 10
run the scenario with
$ roslaunch riskrrt scenario_name.launch
change the planner behavior by modifying the parameters in params/riskrrt_params.yaml



**provided nodes**

controller: a path following node for differential drive robots
            subscribed topics: trajectory (riskrrt/Trajectory) 
                               robot pose (geometry_msgs/PoseWithCovarianceStamped)
            published topics: controller feedback, true if robot is on trajectory (std_msgs/Bool)
                              velocity commands (geometry_msgs/Twist)
goal_pub: automatic goal publisher
          args: pose x (m), pose y (m) with respect to map
          published topics: goal (geometry_msgs/PoseStamped)
og_builder*: some occupany grid array publishers
             args: number of humans, human speed (m/s)
             subscribed topics: map (nav_msgs/OccupancyGrid)
                                human odometry (nav_msgs/Odometry)
             published topics: human velocity (geometry_msgs/Twist)
                               occupancy grid array (riskrrt/OccupancyGridArray)
riskrrt_planner: the riskrrt planner
         subscribed topics: occupancy grid array (riskrrt/OccupancyGridArray)
                            goal (geometry_msgs/PoseStamped)
                            controller feedback (std_msgs/Bool)
                            robot pose (geometry_msgs/PoseWithCovarianceStamped)
                            robot odometry (nav_msgs/Odometry)
         published topics: trajectory (riskrrt/Trajectory)



**publishing your own map**

use the maxDepth and timeStep parameters from params/riskrrt_params.yaml
-maxDepth should be the occupancy grid array size
-timeStep should be the time gap between each grid
