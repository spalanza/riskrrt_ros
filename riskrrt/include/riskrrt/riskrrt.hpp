#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <riskrrt/PoseTwistStamped.h>
#include <riskrrt/Trajectory.h>
#include <nav_msgs/OccupancyGrid.h>
#include <riskrrt/OccupancyGridArray.h>
#include <vector>
#include <tf/tf.h>

using namespace std;

struct Params{
	double timeStep;//time step between each node (s)
  int maxDepth;//maximum depth a node can have
  int nv;//discretization of the robot's linear speed, used to create the set of possible controls from a node
  int nphi;//discretization of the robot's angular speed, used to create the set of possible controls from a node
  double threshold;//maximum risk value a node can have to be considered free
  double socialWeight;//weight of the risk component in the computation of the node's score
  double rotationWeight;//weight of the node's orientation toward the goal (compared to the distance)
  double growTime;//time allocated for the tree growth between map updates (s) /!\: issue forces growtime < timestep
  double bias;//percentage of the time the final goal is chosen as random goal
  double goalTh;//maximum distance the robot can be from the final goal to consider the goal reached (m)
  double windowSize;//size of the window from which to pick a random goal (m)
  double robotLength;//robot lenght (m) /!\: the wheel axis is assumed to be in the middle
  double robotWidth;//robot width (m)
  double vMin;//minimum robot linear speed (m/s), set it to 0 or a negative value to allow reverse
  double vMax;//maximum robot linear speed (m/s)
  double accMax;//maximum linear acceleration (m/s^2)
  double omegaMax;//maximum angular speed (rad/s)
  double accOmegaMax;//maximum angular acceleration (rad/s^2)
};

struct custom_pose{//function provided by the tf library to get yaw from quaternion is terrible, hence this.
  double x;//in meters
  double y;//in meters
  double theta;//in radians (-PI, PI)
};

struct Control{
  geometry_msgs::Twist twist;
  bool open;//whether the control has already been tested or not (doesn't matter if the resulting node was in collision or was effectively added to the tree)
};

struct Node{
  ros::Time time;//time at which node is valid
  custom_pose pose;//pose of the node
  geometry_msgs::Twist vel;//velocity the robot should have at this node
  Node* parent;//parent node
  vector<Node*> sons;//list of sons
  vector<Control> possible_controls;//list of all possible controls from that node considering nv, nphi, velocities and acceleration limits
  double risk;//node risk (=/= node weight, we don't consider distance to goal)
  int depth;//depth of node in the tree
  bool isFree;//false if the node is in collision (the risk is higher than the threshold set by user)
  bool isOpen;//true if at least one control among all possible controls remain open
};


class RRT{
public:
  
  //////////////////ATTRIBUTES//////////////////
  ros::NodeHandle nodeHandle;
  custom_pose robot_pose;//pose of the robot
  geometry_msgs::Twist robot_vel;//velocity of the robot
  Node* root;//tree root
  Node* best_node;//best node to reach final goal (=/= best node to reach random goal)
  vector<Node*> candidate_nodes;//all the nodes that are candidates to grow, list is created and updated during growing phase, then deleted
  custom_pose final_goal;//final goal set by user (=/= random goal)
  bool goal_received;//flag to know when a new goal has been received
  bool robot_on_traj;//is the robot close enough to its theoretical pose, flag for reinit
  riskrrt::OccupancyGridArray og_array;//array of maps, time of map = timeStep * position in array. TODO: multiple gridArray
  riskrrt::Trajectory traj_msg;//trajectory msg for controller
  vector<Node*> traj;//trajectory with pointer to the nodes, used to update tree
  Params params;//list of params
  //tree markers
  visualization_msgs::MarkerArray node_markers;
  visualization_msgs::MarkerArray path_markers;
  //subscribers
  ros::Subscriber controllerFeedbackSubscriber;
  ros::Subscriber ogArraySubscriber;
  ros::Subscriber poseSubscriber;
  ros::Subscriber goalSubscriber;
  ros::Subscriber odomSubscriber;
  
  
  //////////////////FUNCTIONS//////////////////
  RRT(Params params);
  ~RRT();
  void init();//initialisation, creates a root at robot location
  void grow();//adds 1 new node to the tree
  void grow_to_goal(Node* best_node, custom_pose goal);//TODO
  void update();//updates the tree, deletes unreachable nodes and updates remaining nodes
  void updateNodes();//depth first update of all nodes in tree, also creates node markers
  void deleteUnreachableNodes(Node* new_root);//deletes the tree minus the subtree defined by new_root
  Node* chooseBestNode(custom_pose goal);//returns the best node among candidates nodes with respect to goal
  void findPath();//select the best node with respect to the final goal and creates the path from root to that node
  bool isGoalReached();//returns true if robot close enough to the goal (threshold set by user)
  custom_pose chooseRandomGoal();//returns a random pose inside a user defined window
  double computeNodeWeight(Node* node, custom_pose goal);//returns the node score given the goal to reach (final or random)
  void extend(Node* node, custom_pose random_goal);//extends the tree from by creating a new node given an already existing node and a goal 
  vector<Control> discretizeVelocities(Node* node);//returns a list of all possible controls for a node
  double trajLength(custom_pose pose, custom_pose goal);//return a distance from pose to goal based on both euclidian distance and orientation
  custom_pose robotKinematic(custom_pose pose, Control control);//get the pose of the new node for a differential robot
  double computeControlScore(custom_pose expected_pose, custom_pose random_goal);//returns control score given the pose that would be obtained by applying the control during timestep and a goal
  double computeNodeRisk(Node* node);//read the risks from the grid and compute a global risk
  
  //bunch of functions to help switching between coordinates types
  int gridIndexFromPose(custom_pose pose);
  int gridIFromPose(custom_pose pose);
  int gridJFromPose(custom_pose pose);
  int gridIndexFromCoord(int i, int j);
  int gridIFromIndex(int index);
  int gridJFromIndex(int index);
  custom_pose poseFromGridIndex(int index);
  custom_pose poseFromGridCoord(int i, int j);
  
  //subscribers initializations and callbacks
  void controllerFeedbackCallback(const std_msgs::Bool::ConstPtr& msg);
  void initcontrollerFeedbackSub();
  void ogArrayCallback(const riskrrt::OccupancyGridArray::ConstPtr& msg);
  void initOgArraySub();
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void initOdomSub();
  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void initPoseSub();
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void initGoalSub();
  
};
