#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <riskrrt/OccupancyGridArray.h>
#include <riskrrt/PoseTwistStamped.h>
#include <riskrrt/Trajectory.h>
#include <vector>
#include <iostream>
#include <tf/tf.h>
#include <algorithm>
#include <visualization_msgs/MarkerArray.h>
#include <people_msgs/People.h>
#include <people_msgs/Person.h>

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
//double arg_vel;
//int ped_num;
vector<custom_pose> ped_poses, ped_speeds;
custom_pose ped_pose, ped_speed, initial_pose, temp_pose;

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
  
    //for(i=0;i<grid.info.width;i++){
      //for(j=0;j<grid.info.height;j++){
        //if(grid.data[gridIndexFromCoord(i, j)] == 100){
          //for(x=i-20;x<i+21;x++){
            //for(y=j-20;y<j+21;y++){
              //if(x>0 && x<grid.info.width && y>0 && y<grid.info.height){
                //grid.data[gridIndexFromCoord(x, y)] = max((int)grid.data[gridIndexFromCoord(x, y)], (int)floor(100.0 / (sqrt(pow(x-i,2) + pow(y-j,2))/2 + 1)));
              //}
            //}
          //}
        //}
      //}
    //}
  
  og_array.array.clear();
  for(i=0;i<nbMap;i++){
    og_array.array.push_back(grid);
  }
  
}

void pedposeCallback(const people_msgs::People::ConstPtr& msg)
{
  
  int i, j, k;
  int ped_grid_index;
  int ped_grid_i;
  int ped_grid_j;
  
  int max_i, min_i;
  int max_j, min_j;
  
  int closest_index;
  double closest_distance;
  
  
  closest_distance = 10000.0;
  closest_index = -1;
  for(i=0;i<msg->people.size();i++){
    if(closest_distance > sqrt(pow(msg->people[i].position.x, 2) + pow(msg->people[i].position.y, 2))){
      closest_distance = sqrt(pow(msg->people[i].position.x, 2) + pow(msg->people[i].position.y, 2));
      closest_index = i;
    }
  }
  
  ped_pose.x = msg->people[closest_index].position.x;
  ped_pose.y = msg->people[closest_index].position.y;
  ped_speed.x = msg->people[closest_index].velocity.x;
  ped_speed.y = msg->people[closest_index].velocity.y;
  
  ped_poses.push_back(ped_pose);
  if(ped_poses.size() > 20){
    ped_poses.erase(ped_poses.begin());
  }
  
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "human";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = ped_pose.x;
  marker.pose.position.y = ped_pose.y;
  marker.pose.position.z = 0;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 2.0;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.lifetime = ros::Duration(0.3);
  
  for(k=0;k<nbMap;k++){
    
    temp_pose.x = ped_pose.x + k*timeStep*msg->people[closest_index].velocity.x;
    temp_pose.y = ped_pose.y + k*timeStep*msg->people[closest_index].velocity.y;
    ped_grid_i = gridIFromPose(temp_pose);
    ped_grid_j = gridJFromPose(temp_pose);
    
    min_i = max(ped_grid_i - 5, 0);
    max_i = min(ped_grid_i + 5, (int)grid.info.width);
    min_j = max(ped_grid_j - 5, 0);
    max_j = min(ped_grid_j + 5, (int)grid.info.height);
    
    for(i=min_i ; i<=max_i ; i++){
      for(j=min_j ; j<=max_j ; j++){
        og_array.array[k].data[gridIndexFromCoord(i,j)] = max((int)og_array.array[k].data[gridIndexFromCoord(i,j)], 100);
      }
    }
  }
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "og_builder_leg_detector");

  ros::NodeHandle n;
  
  if (argc != 3){
    ROS_INFO("USAGE: og_builder_leg_detector [initial pose x] [initial pose y]");
    return 0;
  }
  
  n.param("maxDepth", nbMap, 10);
  n.param("timeStep", timeStep, 0.5);
  
  //geometry_msgs::Twist ped_vel;
  //arg_vel = atof(argv[2]);
  //ped_num = atoi(argv[1]);
  //ped_vel.linear.x = arg_vel;
  
  initial_pose.x = atoi(argv[1]);
  initial_pose.y = atof(argv[2]);
  
  //stringstream ss;
  //string str;
  int i;

  ros::Subscriber og_sub = n.subscribe("map", 1, pedposeCallback);
  //ped pose subs
  //vector<ros::Subscriber> sub_vect;
  //ros::Subscriber sub;
  //for(i=0;i<ped_num;i++){
    //ss << "/robot_" << i+1 << "/base_pose_ground_truth";
    //str = ss.str();
    //sub = n.subscribe(str, 10, pedposeCallback);
    //sub_vect.push_back(sub);
    //ss.clear();//clear any bits set
    //ss.str(string());
  //}
  ros::Subscriber people_sub = n.subscribe("people", 1, OGCallback);
  
  ros::Publisher test_pub = n.advertise<nav_msgs::OccupancyGrid>("test_map", 1);
  ros::Publisher og_pub = n.advertise<riskrrt::OccupancyGridArray>("ogarray", 1);
  //ped speed pubs
  //vector<ros::Publisher> pub_vect;
  //ros::Publisher pub;
  //for(i=0;i<ped_num;i++){
    //ss << "/robot_" << i+1 << "/cmd_vel";
    //str = ss.str();
    //pub = n.advertise<geometry_msgs::Twist>(str, 1);
    //pub_vect.push_back(pub);
    //ss.clear();//clear any bits set
    //ss.str(string());
  //}

  ros::Rate loop_rate(10);
  ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>( "visualization_marker", 0 );

  while (ros::ok())
  {
    
    ros::spinOnce();
    
    og_pub.publish(og_array);
    test_pub.publish(og_array.array[nbMap-1]);
    og_array.array.clear();
    for(i=0;i<nbMap;i++){
      og_array.array.push_back(grid);
    }
    //modified_grid = grid;
    
    //for(i=0;i<ped_num;i++){
      //pub_vect[i].publish(ped_vel);
    //}
    
    vis_pub.publish( marker );

    loop_rate.sleep();
  }
  return 0;
}
