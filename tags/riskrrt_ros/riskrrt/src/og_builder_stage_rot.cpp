#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <riskrrt/OccupancyGridArray.h>
#include <riskrrt/PoseTwistStamped.h>
#include <riskrrt/Trajectory.h>
#include <vector>
#include <iostream>
#include <tf/tf.h>
#include <algorithm>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

struct custom_pose{//ros pose msgs use quaternion and the function provided by the tf library to get yaw from quaternion is terrible, hence this.
  double x;//in meters
  double y;//in meters
  double theta;//in radians (-PI, PI)
};

nav_msgs::OccupancyGrid grid;
//nav_msgs::OccupancyGrid modified_grid;
riskrrt::OccupancyGridArray og_array;
int nbMap;
double timeStep;
double arg_vel;
int ped_num;
custom_pose temp_pose, ped_pose;
bool straight;

visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;


int gridIFromPose(custom_pose pose){
  return (int)round((pose.x - grid.info.origin.position.x) / grid.info.resolution);
}

int gridJFromPose(custom_pose pose){
  return (int)round((pose.y - grid.info.origin.position.y) / grid.info.resolution);
}

int gridIndexFromCoord(int i, int j){
  return i + grid.info.width * j;
}

int gridIFromIndex(int index){
  return  index % grid.info.width;
}

int gridJFromIndex(int index){
  return floor(index / grid.info.width);
}

custom_pose poseFromGridCoord(int i, int j){
  custom_pose pose;
  pose.x = grid.info.resolution * i + grid.info.origin.position.x;
	pose.y = grid.info.resolution * j + grid.info.origin.position.y;
  return pose;
}

int gridIndexFromPose(custom_pose pose){
  int index, i, j;
  i = gridIFromPose(pose);
  j = gridJFromPose(pose);
  index = gridIndexFromCoord(i, j);
  return index;
}

custom_pose poseFromGridIndex(int index){//not really a pose but rather a point
  int i, j;
  custom_pose pose;
  
  i = gridIFromIndex(index);
  j = gridJFromIndex(index);
  pose = poseFromGridCoord(i, j);
  
  return pose;
}

void OGCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  int i;
  
  grid = *msg;
  //modified_grid = grid;
  og_array.array.clear();
  for(i=0;i<nbMap;i++){
    og_array.array.push_back(grid);
  }
  
}

void pedposeCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  
  int i, j, k;
  int ped_grid_index;
  int ped_grid_i;
  int ped_grid_j;
  
  int max_i, min_i;
  int max_j, min_j;
  
  ped_pose.x = msg->pose.pose.position.x;
  ped_pose.y = msg->pose.pose.position.y;
  ped_pose.theta = tf::getYaw(msg->pose.pose.orientation);
  
  
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time();
  marker.ns = "humans";
  marker.id = rand();
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = ped_pose.x;
  marker.pose.position.y = ped_pose.y;
  marker.pose.position.z = 0;
  //marker.pose.orientation = msg->pose.pose.orientation;
  if(straight){
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(ped_pose.theta + 1.57);
  }
  else marker.pose.orientation = tf::createQuaternionMsgFromYaw(ped_pose.theta + 1.57 + M_PI);
  marker.scale.x = 0.4*0.0254;
  marker.scale.y = 0.4*0.0254;
  marker.scale.z = 0.4*0.0254;
  //marker.color.a = 1.0; // Don't forget to set the alpha!
  //marker.color.r = 1.0;
  //marker.color.g = 1.0;
  //marker.color.b = 1.0;
  marker.lifetime = ros::Duration(0.125);
  marker.mesh_use_embedded_materials = true;
  marker.mesh_resource = "package://riskrrt/meshes/female2/models/female2.dae";
  marker_array.markers.push_back(marker);
  
  for(k=0;k<nbMap;k++){
    
    temp_pose.x = ped_pose.x + k*timeStep*arg_vel*cos(ped_pose.theta);
    temp_pose.y = ped_pose.y + k*timeStep*arg_vel*sin(ped_pose.theta);
    ped_grid_i = gridIFromPose(temp_pose);
    ped_grid_j = gridJFromPose(temp_pose);
    
    min_i = max(ped_grid_i - 5, 0);
    max_i = min(ped_grid_i + 5, (int)grid.info.width);
    min_j = max(ped_grid_j - 5, 0);
    max_j = min(ped_grid_j + 5, (int)grid.info.height);
    
    for(i=min_i ; i<=max_i ; i++){
      for(j=min_j ; j<=max_j ; j++){
        og_array.array[k].data[gridIndexFromCoord(i,j)] = 100;
      }
    }
  }
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "og_builder_stage");

  ros::NodeHandle n;
  
  if (argc != 3){
    ROS_INFO("USAGE: og_builder_stage [number of pedestrians] [speed]");
    return 0;
  }
  
  n.param("maxDepth", nbMap, 10);
  n.param("timeStep", timeStep, 0.5);
  
  geometry_msgs::Twist ped_vel;
  arg_vel = atof(argv[2]);
  ped_num = atoi(argv[1]);
  
  stringstream ss;
  string str;
  int i;

  ros::Subscriber og_sub = n.subscribe("map", 1, OGCallback);
  //ped pose subs
  vector<ros::Subscriber> sub_vect;
  ros::Subscriber sub;
  for(i=0;i<ped_num;i++){
    ss << "/robot_" << i+1 << "/base_pose_ground_truth";
    str = ss.str();
    sub = n.subscribe(str, 10, pedposeCallback);
    sub_vect.push_back(sub);
    ss.clear();//clear any bits set
    ss.str(string());
  }
  
  ros::Publisher test_pub = n.advertise<nav_msgs::OccupancyGrid>("test_map", 1);
  ros::Publisher og_pub = n.advertise<riskrrt::OccupancyGridArray>("ogarray", 1);
  //ped speed pubs
  vector<ros::Publisher> pub_vect;
  ros::Publisher pub;
  for(i=0;i<ped_num;i++){
    ss << "/robot_" << i+1 << "/cmd_vel";
    str = ss.str();
    pub = n.advertise<geometry_msgs::Twist>(str, 1);
    pub_vect.push_back(pub);
    ss.clear();//clear any bits set
    ss.str(string());
  }

  ros::Rate loop_rate(10);
  ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>( "human_marker", 0 );
  
  ros::Time begin;
  begin = ros::Time::now();
  ros::Time present;
  ros::Duration dur1(30.0);
  ros::Duration dur2(60.0);
  straight = true;

  while (ros::ok())
  {
    
    present = ros::Time::now();
    if(present-begin <= dur1){
      straight=true;
    }
    else if(present-begin <= dur2){
      straight=false;
    }
    else begin=ros::Time::now();

    if(straight){
      arg_vel = fabs(arg_vel);
      ped_vel.linear.x = arg_vel;
    }
    else{
      arg_vel = -fabs(arg_vel);
      ped_vel.linear.x = arg_vel;
    }
    
    marker_array.markers.clear();
    
    ros::spinOnce();
    
    og_pub.publish(og_array);
    test_pub.publish(og_array.array[nbMap-1]);
    og_array.array.clear();
    for(i=0;i<nbMap;i++){
      og_array.array.push_back(grid);
    }
    //modified_grid = grid;
    
    for(i=0;i<ped_num;i++){
      pub_vect[i].publish(ped_vel);
    }
    
    vis_pub.publish(marker_array);
    marker_array.markers.clear();

    loop_rate.sleep();
  }
  return 0;
}

