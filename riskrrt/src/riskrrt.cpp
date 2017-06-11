#include "riskrrt/riskrrt.hpp"
#include <cmath>
#include <algorithm>

using namespace std;

RRT::RRT(Params params){
  this->params = params;
  
}

RRT::~RRT(){
  
}

//returns the node score given the goal to reach (final or random)
//the bigger the weight, the better the node
double RRT::computeNodeWeight(Node* node, custom_pose goal){

  double weight;
  
  weight = 1.0 / (params.socialWeight * node->risk + trajLength(node->pose, goal)) * 1e-9;
  
  return weight;
}

//return a distance from pose to goal based on both euclidian distance and orientation
double RRT::trajLength(custom_pose pose, custom_pose goal){
  
  double pose_euclide_distance;
  double root_euclide_distance;
  double position_improvement;
  double rotation_diff;
  double distance_from_goal;
  
  //distance from pose to goal
  pose_euclide_distance = sqrt(pow(pose.x - goal.x, 2) + pow(pose.y - goal.y, 2));
  //distance from root to goal
  root_euclide_distance = sqrt(pow(root->pose.x - goal.x, 2) + pow(root->pose.y - goal.y, 2));
  //position improvement with respect to the goal
  //ratio < 1 means we are getting closer to the goal
  //ratio > 1 means we are getting further away from the goal
  position_improvement = pose_euclide_distance / root_euclide_distance;
  //angle difference between pose orientation and the pose-goal vector
  rotation_diff = atan2(goal.y - pose.y, goal.x - pose.x) - pose.theta;
  //angle must be in [-PI;PI]
  if(rotation_diff > M_PI){
    rotation_diff -= 2 * M_PI;
  }
  if(rotation_diff < -M_PI){
    rotation_diff += 2 * M_PI;
  }
  distance_from_goal = position_improvement + params.rotationWeight * fabs(rotation_diff);
  
  return distance_from_goal;
}

//extends the tree from by creating a new node given an already existing node and a goal
void RRT::extend(Node* node, custom_pose random_goal){
  Node* new_node;
  new_node = new Node;
  custom_pose expected_pose;
  int best_control_index;
  double control_score, best_control_score;
  int i, j;
  bool node_still_open;
  
  best_control_score = 0.0;
  //choose the best control among the (still open) possible controls
  for(i=0; i<node->possible_controls.size(); i++){
    if(node->possible_controls[i].open){
      //compute what pose would be obtained with that control
      expected_pose = robotKinematic(node->pose, node->possible_controls[i]);
      //compute the score for that pose
      control_score = computeControlScore(expected_pose, random_goal);
      if(control_score >= best_control_score){
        best_control_score = control_score;
        best_control_index = i;
      }
    }
  }
  
  //create the new node from the best control
  new_node->time = node->time + ros::Duration(params.timeStep);
  new_node->pose = robotKinematic(node->pose, node->possible_controls[best_control_index]);
  new_node->vel = node->possible_controls[best_control_index].twist;
  new_node->parent = node;
  new_node->sons.clear();
  new_node->possible_controls = discretizeVelocities(new_node);
  new_node->depth = node->depth + 1;
  new_node->risk = computeNodeRisk(new_node);
  new_node->isFree = (new_node->risk <= params.threshold);
  new_node->isOpen = true;
  //add new node to the tree (even if it is in collision, the candidate nodes vector is here to sort the nodes)
  node->sons.push_back(new_node);
  //close the control used to create the new node so it is impossible to create a duplicate node later
  node->possible_controls[best_control_index].open = false;
  
  //add the newly created node to the candidates vector (if it is free) to be taken into account for future expansion during the same growing phase
  if(new_node->isFree && new_node->depth < params.maxDepth){
    candidate_nodes.push_back(new_node);
  }
  
  //check if the last opened control was used and, if so, close the node so that it won't be selected as best node again
  node_still_open = false;
  for(j=0; j<node->possible_controls.size(); j++){
    node_still_open = node_still_open || node->possible_controls[j].open;
  }
  node->isOpen = node_still_open;
}

//read the risks from the grid and compute a global risk
double RRT::computeNodeRisk(Node* node){
  
  //robot's footprint is assumed to be a rectangle
	custom_pose front_left;
  custom_pose front_right;
  custom_pose rear_left;
  custom_pose rear_right;
	int grid_front_left_x, grid_front_left_y;
	int grid_front_right_x, grid_front_right_y;
	int grid_rear_left_x, grid_rear_left_y;
	int grid_rear_right_x, grid_rear_right_y;
	double l, w;
  double node_theta;
	vector<int> grid_cells;
  int i, j;
  double risk, max_risk;
  int grid_max_x, grid_max_y, grid_min_x, grid_min_y;
  
  //the wheel axis is assumed to be in the middle
  l = params.robotLength/2.0;
  w = params.robotWidth/2.0;
	
	//computing the poses of each corner of the robot's footprint
	front_left.x = node->pose.x + (l * cos(node->pose.theta) + w * cos(node->pose.theta + M_PI/2.0));
	front_left.y = node->pose.y + (l * sin(node->pose.theta) + w * sin(node->pose.theta + M_PI/2.0));
	front_right.x = node->pose.x + (l * cos(node->pose.theta) + w * cos(node->pose.theta - M_PI/2.0));
	front_right.y = node->pose.y + (l * sin(node->pose.theta) + w * sin(node->pose.theta - M_PI/2.0));
	rear_left.x = node->pose.x + (l * cos(node->pose.theta + M_PI) + w * cos(node->pose.theta  + M_PI - M_PI/2.0));
	rear_left.y = node->pose.y + (l * sin(node->pose.theta + M_PI) + w * sin(node->pose.theta  + M_PI - M_PI/2.0));
	rear_right.x = node->pose.x + (l * cos(node->pose.theta + M_PI) + w * cos(node->pose.theta + M_PI + M_PI/2.0));
	rear_right.y = node->pose.y + (l * sin(node->pose.theta + M_PI) + w * sin(node->pose.theta + M_PI + M_PI/2.0));
	
	//the corners poses in grid coordinates
  grid_front_left_x = gridIFromPose(front_left);
  grid_front_left_y = gridJFromPose(front_left);
  grid_front_right_x = gridIFromPose(front_right);
  grid_front_right_y = gridJFromPose(front_right);
  grid_rear_left_x = gridIFromPose(rear_left);
  grid_rear_left_y = gridJFromPose(rear_left);
  grid_rear_right_x = gridIFromPose(rear_right);
  grid_rear_right_y = gridJFromPose(rear_right);
	
	//testing: simple bounding box TODO: exact footprint
	grid_max_x = max(grid_front_left_x, max(grid_front_right_x, max(grid_rear_left_x, grid_rear_right_x)));
	grid_max_y = max(grid_front_left_y, max(grid_front_right_y, max(grid_rear_left_y, grid_rear_right_y)));
	grid_min_x = min(grid_front_left_x, min(grid_front_right_x, min(grid_rear_left_x, grid_rear_right_x)));
	grid_min_y = min(grid_front_left_y, min(grid_front_right_y, min(grid_rear_left_y, grid_rear_right_y)));
	
  //creating a list of all the grid cells within the robot's footprint
	for(i=grid_min_x ; i<=grid_max_x ; i++){
		for(j=grid_min_y ; j<=grid_max_y ; j++){
			if (i >=0 && i < (int)og_array.array[0].info.width && j >=0 && j< (int)og_array.array[0].info.height){
				grid_cells.push_back(gridIndexFromCoord(i,j));
			}
		}
	}
  
  //going through all the cells in robot footprint and getting the maximum risk
  max_risk = 0.0;
  for(i=0; i<grid_cells.size(); i++){	  
    risk = og_array.array[node->depth].data[grid_cells[i]];
    max_risk = max(risk, max_risk);
  }
  
  //risk propagation from a node to his sons if their risk is lower
  if(node->parent != NULL){
    max_risk = max(max_risk, node->parent->risk);
  }
  
	return max_risk;
}

//returns control score given the pose that would be obtained by applying the control during timestep and a goal
//the bigger the score, the better the control
double RRT::computeControlScore(custom_pose expected_pose,  custom_pose random_goal){
  double distance_from_random_goal;
  double score;
  
  distance_from_random_goal = trajLength(expected_pose, random_goal);
  score = 1.0 / distance_from_random_goal + 1e-9;
  
  return score;
}

//initialisation, creates a root at robot location
void RRT::init(){
  //creating a new node at the robot current location, with its current speed
  Node* node;
  node = new Node;
  node->time = ros::Time::now();
  node->pose = robot_pose;
  node->vel = robot_vel;
  node->parent = NULL;
  node->sons.clear();
  node->possible_controls = discretizeVelocities(node);
  node->isOpen = true;
  node->depth = 0;
  node->risk = computeNodeRisk(node);
  node->isFree = (node->risk <= params.threshold);
  //set this new node as the tree root
  root = node;
  //set root as the best node to reach the final goal (this is needed to set the window in order to choose the next random goal)
  best_node = node;
  //the tree root is the first candidate to grow
  candidate_nodes.clear();
  candidate_nodes.push_back(root);
}

//adds 1 new node to the tree
void RRT::grow(){
  Node* best_node_to_grow;
  custom_pose random_goal;
  //choosing a random point in window
  random_goal = chooseRandomGoal();
  //choosing the best node within the tree to grow toward the random point
  best_node_to_grow = chooseBestNode(random_goal);
  //checking if there is an actual best node (in some situations, it is possible to have none. for instance, if root is in collision)
  if(best_node_to_grow != NULL){
    //extend the tree from the best node to the random point by choosing the best control
    extend(best_node_to_grow, random_goal);
  }
  else ROS_INFO("ROBOT STUCK");
}

void RRT::grow_to_goal(Node* best_node, custom_pose goal){
  //TODO
}

//returns a random pose inside a user defined window
custom_pose RRT::chooseRandomGoal(){
  double window_size;
  double random_score;
  int window_size_grid;
  int x_min_limit, x_max_limit, y_min_limit, y_max_limit;
  int random_x, random_y;
  custom_pose random_goal;
  
  //getting a random number between 0 and 99 for bias
  random_score = (rand() % 100)/100.0;
  //computing the window size in grid cells
  window_size_grid = (int)floor(params.windowSize / og_array.array[0].info.resolution);
  //reducing the window size if it exceeds the map dimensions
  x_min_limit = max(gridIFromPose(best_node->pose) - window_size_grid, 0);
  x_max_limit = min(gridIFromPose(best_node->pose) + window_size_grid, (int)og_array.array[0].info.width);
  y_min_limit = max(gridJFromPose(best_node->pose) - window_size_grid, 0);
  y_max_limit = min(gridJFromPose(best_node->pose) + window_size_grid, (int)og_array.array[0].info.height);
  if(random_score > params.bias){
    //choosing random position within limits (window or map edges)
    random_x = x_min_limit + rand() % (x_max_limit - x_min_limit);
    random_y = y_min_limit + rand() % (y_max_limit - y_min_limit);
    //same position but in map coordinates
    random_goal = poseFromGridCoord(random_x, random_y);
  }
  else{
    //choosing the final goal as random goal with a set probability (bias)
    random_goal = final_goal;
  }
  return random_goal;
}

//updates the tree, deletes unreachable nodes and updates remaining nodes
void RRT::update(){
  
  
  int index;
  ros::Time present;
  
  present = ros::Time::now();
  
	if(traj.size() <= 1){
    index = -1;
	}
	else if(traj.front()->time > present){
    index = -1;
	}
	else if(traj.back()->time < present){
		index = -1;
	}
  else{
    index = floor((present - traj.front()->time).toSec() / params.timeStep);
  }
  
  //to delete the unreachable nodes, there has to be an actual trajectory with at least 2 nodes (root and one other node)
  //if(traj.size() > 1){
    //if(ros::Time::now() >= traj[1]->time){
    if(index != -1 && index != 0){
      deleteUnreachableNodes(traj[index]);
      //making the first node in trajectory the new root
      root = traj[index];
      root->depth = 0;
      root->parent = NULL;
    }
    //}
  //}
  //update remaining tree
  updateNodes();
}

//depth first update of all nodes in tree, also creates node markers
void RRT::updateNodes(){
  Node* current_node;
  vector<Node*> node_stack;
  int i;
  int nb_nodes;
  visualization_msgs::Marker node_marker;
  
  nb_nodes = 0;
  candidate_nodes.clear();
  
  //robot is always somewhere between root and the first node of the trajectory
  //thus, when the robot is moving, root and its descendants (except the subtree defined by the first node of the trajectory) are unreachable
  //there is no need in updating them or adding them in the candidate nodes vector
  //if(traj.size() > 1){
    //node_stack.push_back(traj[1]);
  //}
  //else{
    node_stack.push_back(root);
  //}
  while(!node_stack.empty()){
    current_node = node_stack.back();
    node_stack.pop_back();
    if(!current_node->sons.empty()){
      for(i=0; i<current_node->sons.size(); i++){
        node_stack.push_back(current_node->sons[i]);
      }
    }
    //update the depth
    if(current_node->parent != NULL){
      current_node->depth = current_node->parent->depth + 1;
    }
    current_node->risk = computeNodeRisk(current_node);
    current_node->isFree = (current_node->risk <= params.threshold);
    //add the "good" nodes to the candidate nodes vector
    if(current_node->isFree && current_node->depth < params.maxDepth && current_node->isOpen){
      candidate_nodes.push_back(current_node);
    }
    nb_nodes++;
    //rviz marker for nodes
    node_marker.header.frame_id = "/map";
    node_marker.header.stamp = ros::Time::now();
    node_marker.id = nb_nodes;
    node_marker.ns = "tree";
    node_marker.type = visualization_msgs::Marker::SPHERE;
    node_marker.action = visualization_msgs::Marker::ADD;
    node_marker.pose.position.x = current_node->pose.x;
    node_marker.pose.position.y = current_node->pose.y;
    node_marker.pose.position.z = 0.05;
    node_marker.scale.x = 0.1;
    node_marker.scale.y = 0.1;
    node_marker.scale.z = 0.1;
    node_marker.color.r = 0.0f;
    node_marker.color.g = 1.0f;
    node_marker.color.b = 0.0f;
    node_marker.color.a = 1.0;
    node_marker.lifetime = ros::Duration(1.0);
    node_markers.markers.push_back(node_marker);
    
  }
}

//deletes the tree minus the subtree defined by new_root
void RRT::deleteUnreachableNodes(Node* new_root){
  Node* current_node;
  vector<Node*> node_stack;
  int i;
  
  node_stack.push_back(root);
  while(!node_stack.empty()){
    current_node = node_stack.back();
    node_stack.pop_back();
    if(!current_node->sons.empty()){
      for(i=0; i<current_node->sons.size(); i++){
        if(current_node->sons[i] != new_root){
          node_stack.push_back(current_node->sons[i]);
        }
      }
    }
    current_node->sons.clear();
    current_node->parent = NULL;
    current_node->possible_controls.clear();
    delete current_node;
  }
}

//returns the best node among candidates nodes with respect to goal
Node* RRT::chooseBestNode(custom_pose goal){
  
  Node* best_rated_node;
  Node* current_node;
  double best_weight;
  double current_weight;
  int i;
  
  best_rated_node = NULL;
  best_weight = 0.0;
  for(i=0; i<candidate_nodes.size(); i++){
    current_node = candidate_nodes[i];
    current_weight = computeNodeWeight(current_node, goal);
    //node weight is better, node is free and open, node depth is less than maximum depth
    if(best_weight <= current_weight && current_node->isOpen){
      best_rated_node = current_node;
      best_weight = current_weight;
    }
  }
  
  return best_rated_node;
}

//select the best node with respect to the final goal and creates the path from root to that node
void RRT::findPath(){
  riskrrt::PoseTwistStamped pose_twist;
  Node* current_node;
  visualization_msgs::Marker path_marker; 
  
  traj_msg.poses.clear();
  traj.clear();
  //find the best node to go toward the final goal and add it to the trajectory
  best_node = chooseBestNode(final_goal);
  //if there is no best node, pick root
  if(best_node == NULL){
    best_node = root;
  }
  //fill the twist message with node attributes
  pose_twist.pose.position.x = best_node->pose.x;
  pose_twist.pose.position.y = best_node->pose.y;
  pose_twist.pose.orientation = tf::createQuaternionMsgFromYaw(best_node->pose.theta);
  pose_twist.twist = best_node->vel;
  pose_twist.time = best_node->time;
  current_node = best_node;
  traj_msg.poses.push_back(pose_twist);
  traj.push_back(current_node);
  //add all bestnode's ancestor to the trajectory until root is reached
  while(current_node->parent != NULL){
    current_node = current_node->parent;
    pose_twist.pose.position.x = current_node->pose.x;
    pose_twist.pose.position.y = current_node->pose.y;
    pose_twist.pose.orientation = tf::createQuaternionMsgFromYaw(current_node->pose.theta);
    pose_twist.twist = current_node->vel;
    pose_twist.time = current_node->time;
    traj_msg.poses.insert(traj_msg.poses.begin(), pose_twist);
    traj.insert(traj.begin(), current_node);
    
    //create a marker for the nodes in the trajectory
    path_marker.header.frame_id = "/map";
    path_marker.header.stamp = ros::Time::now();
    path_marker.id = rand();
    path_marker.ns = "tree";
    path_marker.type = visualization_msgs::Marker::SPHERE;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.pose.position.x = current_node->pose.x;
    path_marker.pose.position.y = current_node->pose.y;
    path_marker.pose.position.z = 0.06;
    path_marker.scale.x = 0.1;
    path_marker.scale.y = 0.1;
    path_marker.scale.z = 0.1;
    path_marker.color.r = 1.0f;
    path_marker.color.g = 0.0f;
    path_marker.color.b = 0.0f;
    path_marker.color.a = 1.0;
    path_marker.lifetime = ros::Duration(1.0);
    path_markers.markers.push_back(path_marker);
    
  }
  //set the trajectory flag to stop robot from executing deprecated trajectories if this trajectory is empty
  traj_msg.exists.data = (traj_msg.poses.size() > 1);  
}

//returns true if robot close enough to the goal (threshold set by user)
bool RRT::isGoalReached(){
  //this is a distance criteria, orientation is not taken into account
  return (sqrt(pow(root->pose.x - final_goal.x, 2) + pow(root->pose.y - final_goal.y, 2)) < params.goalTh);
}

//returns a list of all possible controls for a node
vector<Control> RRT::discretizeVelocities(Node* node){
  
  vector<Control> controls;
  Control control;
  double min_linear_vel;
  double max_linear_vel;
  double min_angular_vel;
  double max_angular_vel;
  int i, j;
  double delta_linear, delta_angular;
  
  //compute maximum and minimum velocities that can be reached from that node given the speed and acceleration limits of the robot
  min_linear_vel = node->vel.linear.x - params.accMax * params.timeStep;
  max_linear_vel = node->vel.linear.x + params.accMax * params.timeStep;
  min_angular_vel = node->vel.angular.z - params.accOmegaMax * params.timeStep;//right
  max_angular_vel = node->vel.angular.z + params.accOmegaMax * params.timeStep;//left
  
  //make sure that speed limits are not exceeded
  min_linear_vel = max(params.vMin, min_linear_vel);
  max_linear_vel = min(params.vMax, max_linear_vel);
  min_angular_vel = max(-params.omegaMax, min_angular_vel);
  max_angular_vel = min(params.omegaMax, max_angular_vel);
  
  //create the set of controls
  delta_linear = (max_linear_vel - min_linear_vel) / double(params.nv);
	delta_angular = (max_angular_vel - min_angular_vel) / double(params.nphi);
  for(i=0; i<params.nv; i++){
    control.twist.linear.x = min_linear_vel + i * delta_linear;
    for(j=0; j<params.nphi; j++){
      control.twist.angular.z = min_angular_vel + j * delta_angular;
      control.open = true;
      controls.push_back(control);
    }
  }
  
  return controls;
}

//get the pose of the new node for a differential robot
custom_pose RRT::robotKinematic(custom_pose pose, Control control){
  custom_pose new_pose;
  double rotation_radius;
  double delta_theta, delta_x, delta_y;
  
  if(control.twist.linear.x == 0.0){
    delta_theta = control.twist.angular.z * params.timeStep;
    delta_x = 0.0;
    delta_y = 0.0;
	}
	else if(control.twist.angular.z == 0.0){
    delta_theta = 0.0;
    delta_x = control.twist.linear.x * params.timeStep;
    delta_y = 0.0;
  }
  else{
    rotation_radius = control.twist.linear.x / control.twist.angular.z;
    delta_theta = control.twist.angular.z * params.timeStep;
    delta_x = rotation_radius * sin(delta_theta);
    delta_y = rotation_radius * (1.0 - cos(delta_theta));
  }
  new_pose.x = pose.x + (delta_x * cos(pose.theta) - delta_y * sin(pose.theta));
  new_pose.y = pose.y + (delta_x * sin(pose.theta) + delta_y * cos(pose.theta));
  new_pose.theta = atan2(sin(pose.theta + delta_theta), cos(pose.theta + delta_theta));
  
  return new_pose;
}


//bunch of functions to help switching between coordinates types
int RRT::gridIndexFromPose(custom_pose pose){
  int index, i, j;
  i = gridIFromPose(pose);
  j = gridJFromPose(pose);
  index = gridIndexFromCoord(i, j);
  return index;
}

int RRT::gridIFromPose(custom_pose pose){
  return (int)round((pose.x - og_array.array[0].info.origin.position.x) / og_array.array[0].info.resolution);
}

int RRT::gridJFromPose(custom_pose pose){
  return (int)round((pose.y - og_array.array[0].info.origin.position.y) / og_array.array[0].info.resolution);
}

int RRT::gridIndexFromCoord(int i, int j){
  return i + og_array.array[0].info.width * j;
}

int RRT::gridIFromIndex(int index){
  return  index % og_array.array[0].info.width;
}

int RRT::gridJFromIndex(int index){
  return floor(index / og_array.array[0].info.width);
}

custom_pose RRT::poseFromGridIndex(int index){
  int i, j;
  custom_pose pose;
  
  i = gridIFromIndex(index);
  j = gridJFromIndex(index);
  pose = poseFromGridCoord(i, j);
  
  return pose;
}

custom_pose RRT::poseFromGridCoord(int i, int j){
  custom_pose pose;
  pose.x = og_array.array[0].info.resolution * i + og_array.array[0].info.origin.position.x;
	pose.y = og_array.array[0].info.resolution * j + og_array.array[0].info.origin.position.y;
  return pose;
}
//end of functions to help switching between coordinates types


//subscribers initializations and callbacks
void RRT::controllerFeedbackCallback(const std_msgs::Bool::ConstPtr& msg){
  robot_on_traj = msg->data;
}

void RRT::initcontrollerFeedbackSub(){
  controllerFeedbackSubscriber = nodeHandle.subscribe("/controller_feedback", 1, &RRT::controllerFeedbackCallback, this);
}

void RRT::ogArrayCallback(const riskrrt::OccupancyGridArray::ConstPtr& msg){
  og_array = *msg;
}

void RRT::initOgArraySub(){
  ogArraySubscriber = nodeHandle.subscribe("/ogarray", 1, &RRT::ogArrayCallback, this);
}

void RRT::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  robot_vel = msg->twist.twist;
}

void RRT::initOdomSub(){
  odomSubscriber = nodeHandle.subscribe("/odom", 1, &RRT::odomCallback, this);
}

void RRT::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  robot_pose.x = msg->pose.pose.position.x;
  robot_pose.y = msg->pose.pose.position.y;
  robot_pose.theta = tf::getYaw(msg->pose.pose.orientation);
}

void RRT::initPoseSub(){
  poseSubscriber = nodeHandle.subscribe("/amcl_pose", 1, &RRT::poseCallback, this);
}

void RRT::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  final_goal.x = msg->pose.position.x;
  final_goal.y = msg->pose.position.y;
  final_goal.theta = tf::getYaw(msg->pose.orientation);
  goal_received = true;
}
  
void RRT::initGoalSub(){
  goalSubscriber = nodeHandle.subscribe("goal", 1, &RRT::goalCallback, this);
}
//end of subscribers initializations and callbacks
