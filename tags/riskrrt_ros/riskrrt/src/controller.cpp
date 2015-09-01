#include <ros/ros.h>
#include <riskrrt/PoseTwistStamped.h>
#include <riskrrt/Trajectory.h>//ros only have path msg, no trajectory
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <vector>

using namespace std;

geometry_msgs::Pose robot_pose;
riskrrt::Trajectory trajectory;

//kanayama control law
//returns a corrected control from the robot theoretical, estimated pose and current speed
geometry_msgs::Twist kanayama(geometry_msgs::Pose theoretical_pose, geometry_msgs::Pose estimated_pose, geometry_msgs::Twist control){

  double delta_x, delta_y, delta_theta;
  double error_x, error_y, error_theta;
  double k_x, k_y, k_theta;
  geometry_msgs::Twist corrected_control;

  //difference between theoretical pose (from the kinematic model) and estimated pose (from localization)
	delta_x = theoretical_pose.position.x - estimated_pose.position.x;
	delta_y = theoretical_pose.position.y - estimated_pose.position.y;
	delta_theta = tf::getYaw(theoretical_pose.orientation) - tf::getYaw(estimated_pose.orientation);
  
  //error
	error_x = delta_x * cos(tf::getYaw(estimated_pose.orientation)) + delta_y * sin(tf::getYaw(estimated_pose.orientation));
	error_y = -delta_x * sin(tf::getYaw(estimated_pose.orientation)) + delta_y * cos(tf::getYaw(estimated_pose.orientation));
	error_theta = delta_theta;

  //some coefficient
	k_x = 0.15;
	k_y = 0.15;
	k_theta = 2 * sqrt(k_y);
	
  //corrected control
  corrected_control.linear.x = control.linear.x * cos(error_theta) + k_x * error_x;
  corrected_control.angular.z = control.angular.z + control.linear.x * (k_y * error_y + k_theta * sin(error_theta));
	
	return corrected_control;
}

//returns a control that stops the robot (stops the robot from executing the end of a deprecated trajectory if it cannot find a new one)
geometry_msgs::Twist brake(){
  
  geometry_msgs::Twist corrected_control;
  
  corrected_control.linear.x = 0.0;
  corrected_control.angular.z = 0.0;
    
	return corrected_control;
};

//returns a pose given a starting pose, velocity and duration
//differential drive kinematic
geometry_msgs::Pose robotKinematic(geometry_msgs::Pose pose, geometry_msgs::Twist velocity, double duration){
  geometry_msgs::Pose new_pose;
  double rotation_radius;
  double delta_theta, delta_x, delta_y;
  
  if(velocity.linear.x == 0.0){
    delta_theta = velocity.angular.z * duration;
    delta_x = 0.0;
    delta_y = 0.0;
	}
	else if(velocity.angular.z == 0.0){
    delta_theta = 0.0;
    delta_x = velocity.linear.x * duration;
    delta_y = 0.0;
  }
  else{
    rotation_radius = velocity.linear.x / velocity.angular.z;
    delta_theta = velocity.angular.z * duration;
    delta_x = rotation_radius * sin(delta_theta);
    delta_y = rotation_radius * (1.0 - cos(delta_theta));
  }
  new_pose.position.x = pose.position.x + (delta_x * cos(tf::getYaw(pose.orientation)) - delta_y * sin(tf::getYaw(pose.orientation)));
  new_pose.position.y = pose.position.y + (delta_x * sin(tf::getYaw(pose.orientation)) + delta_y * cos(tf::getYaw(pose.orientation)));
  new_pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(pose.orientation) + delta_theta);
  
  return new_pose;
}

//returns the index of the last visited node in the trajectory
int interpolation(riskrrt::Trajectory trajectory, double timeStep){
  
  int index;
  ros::Time present;
  
  present = ros::Time::now();
  
  //there is only root in the trajectory, wait for an actual trajectory
	if(trajectory.poses.size() <= 1){
    index = -1;
	}
  //trajectory hasn't started yet (root time is in the future)
	else if(trajectory.poses.front().time > present){
    index = -1;
	}
  //trajectory has already ended (last node of trajectory is in the past)
	else if(trajectory.poses.back().time < present){
		index = -1;
	}
  else{
    index = floor((present - trajectory.poses.front().time).toSec() / timeStep);
  }
    
	return index;
}

void trajectoryCallback(const riskrrt::Trajectory::ConstPtr& msg){
  trajectory = *msg;
}

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  robot_pose = msg->pose.pose;
}

int main(int argc, char** argv){
  
  ros::init(argc, argv, "riskrrt_controller");
  
  ros::NodeHandle n;
  
  geometry_msgs::Pose theoretical_pose;
  geometry_msgs::Twist raw_control;
  int index;
  double duration;
  double timeStep;
  geometry_msgs::Twist corrected_control;
  std_msgs::Bool controller_feedback;
  
  n.param("timeStep", timeStep, 0.5);
  
  ros::Publisher controlPublisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  //publish a msg to tell the rrt if the robot is too far from the trajectory
  ros::Publisher controllerFeedbackPublisher = n.advertise<std_msgs::Bool>("controller_feedback",1);
  
  ros::Subscriber trajectorySubscriber = n.subscribe("/traj", 1, trajectoryCallback);
  ros::Subscriber poseSubscriber = n.subscribe("/amcl_pose", 1, poseCallback);
  
  ros::Rate loop_rate(20.0);
  trajectory.exists.data = false;
  
  while(ros::ok()){
    
    ros::spinOnce();
    
    if(trajectory.exists.data){
      
      //what is the last node of the trajectory the robot visited
      index = interpolation(trajectory, timeStep);
      
      if(index != -1){
        
        //time difference between present time and timestamp of the last visited node (s)
        duration = (ros::Time::now() - trajectory.poses[index].time).toSec();
        //where should the robot be on the trajectory at present time
        theoretical_pose = robotKinematic(trajectory.poses[index].pose, trajectory.poses[index+1].twist, duration);
        //compute corrected control
        corrected_control = kanayama(theoretical_pose, robot_pose, trajectory.poses[index+1].twist);
        
        //check the difference between the estimated robot pose and the theoretical pose
        double pose_error = sqrt(pow(theoretical_pose.position.x - robot_pose.position.x, 2) + pow(theoretical_pose.position.y - robot_pose.position.y, 2));
        controller_feedback.data = (pose_error < 0.3);
        
      }
      else{
        corrected_control = brake();
        controller_feedback.data = true;
      }
      
    }
    else{
      corrected_control = brake();
      controller_feedback.data = true;
    }
    
    //publish the corrected control
    controlPublisher.publish(corrected_control);
    //publish a message to tell the rrt if the robot position is ok
    controllerFeedbackPublisher.publish(controller_feedback);
    
    loop_rate.sleep();
  }

  return 0;
}
