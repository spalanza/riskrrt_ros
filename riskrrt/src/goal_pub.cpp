#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <unistd.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "goal_pub");
  ros::NodeHandle n;
  
  if (argc != 3){
    ROS_INFO("USAGE: goal_pub [x] [y]");
    return 0;
  }
  
  ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("goal", 1000);
  
  geometry_msgs::PoseStamped goal_msg;
  goal_msg.pose.position.x = atof(argv[1]);
  goal_msg.pose.position.y = atof(argv[2]);
  
  
  ros::Rate loop_rate(100);
  
  
  while(goal_pub.getNumSubscribers() == 0){
    loop_rate.sleep();
  }
  
  ros::Duration(5.0).sleep();

  goal_pub.publish(goal_msg);


  return 0;
}
