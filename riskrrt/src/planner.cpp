#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <riskrrt/Trajectory.h>
#include "riskrrt/riskrrt.hpp"

using namespace std;

int main(int argc, char **argv){
 
  //Initializing ROS node
  ros::init(argc, argv, "rrt");
  ros::NodeHandle n;
 
  //Reading parameters from yaml file (description in yaml file)
  Params params;
  n.param("timeStep", params.timeStep, 0.5);
  n.param("maxDepth", params.maxDepth, 10);
  n.param("nv", params.nv, 10);
  n.param("nphi", params.nphi, 10);
  n.param("threshold", params.threshold, 0.9);
  n.param("socialWeight", params.socialWeight, 10.0);
  n.param("rotationWeight", params.rotationWeight, 5.0);
  n.param("growTime", params.growTime, 0.1);
  n.param("bias", params.bias, 0.02);
  n.param("goalTh", params.goalTh, 0.5);
  n.param("windowSize", params.windowSize, 10.0);
  n.param("robotLength", params.robotLength, 1.5);
  n.param("robotWidth", params.robotWidth, 0.7);
  n.param("vMin", params.vMin, 0.0);
  n.param("vMax", params.vMax, 0.5);
  n.param("accMax", params.accMax, 0.5);
  n.param("omegaMax", params.omegaMax, 1.0);
  n.param("accOmegaMax", params.accOmegaMax, 1.0);
  
  //maxdepth is the maximum depth a node can have to grow, thus this node can have a son for which we will compute the risk and try to access an inexistent grid layer
  params.maxDepth = params.maxDepth - 1;
  
  //creating rrt 
  RRT* rrt;
  rrt = new RRT(params);
  
  //publishers
  ros::Publisher traj_pub = n.advertise<riskrrt::Trajectory>("traj", 10);
  ros::Publisher node_markers_pub = n.advertise<visualization_msgs::MarkerArray>("node_markers", 10);
  ros::Publisher path_markers_pub = n.advertise<visualization_msgs::MarkerArray>("path_markers", 10);
  
  ros::Publisher robot_marker_pub = n.advertise<visualization_msgs::Marker>( "robot_marker", 0 );
  visualization_msgs::Marker marker;
  ros::Publisher goal_marker_pub = n.advertise<visualization_msgs::Marker>( "goal_marker", 0 );
  visualization_msgs::Marker gmarker;
  
  //subscribers
  rrt->initOgArraySub();
  rrt->initOdomSub();
  rrt->initPoseSub();
  rrt->initGoalSub();
  rrt->initcontrollerFeedbackSub();
  
  //timers
  ros::WallTime growing_start_time;
  ros::WallTime present;
  ros::WallDuration time_spent_growing;
  ros::WallDuration total_time_to_grow(params.growTime);
  
  //constants & variables
  double distance_to_goal_th = params.goalTh;
  int i;
  
  //goal flag to start rrt
  rrt->goal_received = false;
  while (ros::ok())
  {
    
    if(rrt->goal_received)//TODO: always on rrt
    {
      
      //initializing rrt
      rrt->init();
      ROS_INFO("GOAL RECEIVED");
      
      while(!(rrt->isGoalReached()) && ros::ok())//while goal not reached
      {
      
        gmarker.header.frame_id = "/map";
        gmarker.header.stamp = ros::Time();
        gmarker.ns = "goal";
        gmarker.id = 1;
        gmarker.type = visualization_msgs::Marker::SPHERE;
        gmarker.action = visualization_msgs::Marker::ADD;
        gmarker.pose.position.x = rrt->final_goal.x;
        gmarker.pose.position.y = rrt->final_goal.y;
        gmarker.pose.position.z = 0.15;
        gmarker.scale.x = 0.3;
        gmarker.scale.y = 0.3;
        gmarker.scale.z = 0.3;
        gmarker.color.a = 1.0; // Don't forget to set the alpha!
        gmarker.color.r = 0.0;
        gmarker.color.g = 0.0;
        gmarker.color.b = 1.0;
        goal_marker_pub.publish( gmarker );
        
        
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time();
        marker.ns = "robot";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = rrt->robot_pose.x;
        marker.pose.position.y = rrt->robot_pose.y;
        marker.pose.position.z = 0;
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(rrt->robot_pose.theta);
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        //marker.color.a = 1.0; // Don't forget to set the alpha!
        //marker.color.r = 1.0;
        //marker.color.g = 1.0;
        //marker.color.b = 1.0;
        marker.mesh_use_embedded_materials = true;
        marker.mesh_resource = "package://riskrrt/meshes/wheelchair/Wheelchair1.dae";
        robot_marker_pub.publish( marker );
        
        
        growing_start_time = ros::WallTime::now();
        time_spent_growing = ros::WallTime::now() - growing_start_time;
        i=0;
        
        while(time_spent_growing < total_time_to_grow && ros::ok())//while there is still some time left
        {
          //make the tree grow
          rrt->grow();
          i++;
          //update the timer to know how much time we have left to grow
          time_spent_growing = ros::WallTime::now() - growing_start_time;
          
        }//time is up, end of growing part
        
        ////find the best partial path toward the final goal
        //rrt->findPath();
        
        //robot is at the right place at the right time
        if(rrt->robot_on_traj){
          //update the tree with the latest map
          rrt->update();
        }
        else{
          //robot failed to follow the trajectory, start over from current location
          rrt->init();
          ROS_INFO("REINIT");
        }
        
        //find the best partial path toward the final goal
        rrt->findPath();
        
        //publish the trajectory and markers
        traj_pub.publish(rrt->traj_msg);
        node_markers_pub.publish(rrt->node_markers);
        path_markers_pub.publish(rrt->path_markers);
        rrt->node_markers.markers.clear();
        rrt->path_markers.markers.clear();
        
        ros::spinOnce();
        
      }//goal reached
      
      //set the trajectory flag to false and publish it to stop controller
      rrt->traj_msg.exists.data = false;
      traj_pub.publish(rrt->traj_msg);
      ROS_INFO("GOAL REACHED");
      rrt->goal_received = false;
    }
    ros::spinOnce();
    
  }//end ros::ok()

  return 0;
}
