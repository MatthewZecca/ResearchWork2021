#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void leaveCB() {
  MoveBaseClient ac("move_base", true);
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 0.0;
  goal.target_pose.pose.position.y = 0.0;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Returning to docking station");

  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Arrived at docking station");
  else
    ROS_INFO("The jackal failed to return to the docking station");

}

void placeholderAction() {

  leaveCB();

}

void navToCB(const geometry_msgs::PoseStampedConstPtr& goal)
{
  MoveBaseClient ac("move_base", true);
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  
  move_base_msgs::MoveBaseGoal adaptedGoal;
  adaptedGoal.target_pose.header.frame_id = "map";
  adaptedGoal.target_pose.header.stamp = ros::Time::now();

  adaptedGoal.target_pose.pose.position.x = goal->pose.position.x;
  adaptedGoal.target_pose.pose.position.y = goal->pose.position.y;
  adaptedGoal.target_pose.pose.position.z = goal->pose.position.z;
  adaptedGoal.target_pose.pose.orientation.w = goal->pose.orientation.w;

  ROS_INFO("Navigating to the circuit breaker");

  ac.sendGoal(adaptedGoal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Arrived at circuit breaker");
    placeholderAction();
  }
  else
    ROS_INFO("The jackal failed to navigate to the circuit breaker");

}




int main(int argc, char **argv) {

  ros::init(argc, argv, "single_goal_navigation");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("cb_goal", 1000, navToCB);
  ros::spin();

  return 0;
}


