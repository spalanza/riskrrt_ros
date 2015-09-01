#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <dynamic_reconfigure/server.h>
#include <riskrrt/OccupancyGridArray.h>
#include <riskrrt/PoseTwistStamped.h>
#include <riskrrt/Trajectory.h>
#include <vector>
#include <iostream>

using namespace std;

nav_msgs::OccupancyGrid grid;
riskrrt::OccupancyGridArray og_array;
riskrrt::Trajectory traj;
int nbMap;

void OGCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  grid = *msg;
  int i;
  for(i=0;i<nbMap;i++){
    og_array.array.push_back(grid);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "og_builder");

  ros::NodeHandle n;
  
  //n.param("timeStep", timeStep,0.5);
  n.param("maxDepth", nbMap, 10);

  ros::Subscriber og_sub = n.subscribe("map", 1, OGCallback);
  ros::Publisher og_pub = n.advertise<riskrrt::OccupancyGridArray>("ogarray", 1);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    og_pub.publish(og_array);

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}
