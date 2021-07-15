#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

move_base_msgs::MoveBaseGoal goalList [50];
bool isNavigating = false;

void navigate() {

  isNavigating = true;

  MoveBaseClient ac("move_base", true);
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  } 

  for (int i = 0; i < sizeof(goalList); i++) {
    if (goalList[i].target_pose.pose.position.x == 0) break;
    ROS_INFO("Sending goal");
    ac.sendGoal(goalList[i]);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("The jackal reached the goal");
    } else {
      ROS_INFO("The jackal failed to reach the goal");
    }
  }
    
  move_base_msgs::MoveBaseGoal homeGoal; 
  homeGoal.target_pose.header.frame_id = "map";
  homeGoal.target_pose.header.stamp = ros::Time::now();
  homeGoal.target_pose.pose.position.x = 0.0;
  homeGoal.target_pose.pose.position.y = 0.0;
  homeGoal.target_pose.pose.position.z = 0.0;
  homeGoal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(homeGoal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("The jackal reached the docking station");
  } else {
    ROS_INFO("The jackal failed to reach the docking station");
  }

  isNavigating = false;

}

void addGoal(const geometry_msgs::PoseStampedConstPtr& goal) {
  
  move_base_msgs::MoveBaseGoal adaptedGoal;
  adaptedGoal.target_pose.header.frame_id = "map";
  adaptedGoal.target_pose.header.stamp = ros::Time::now();

  adaptedGoal.target_pose.pose.position.x = goal->pose.position.x;
  adaptedGoal.target_pose.pose.position.y = goal->pose.position.y;
  adaptedGoal.target_pose.pose.position.z = goal->pose.position.z;
  adaptedGoal.target_pose.pose.orientation.w = goal->pose.orientation.w;
  
  for (int i = 0; i < sizeof(goalList)-1; i++) {
    if (goalList[i].target_pose.pose.position.x == 0) {
      goalList[i] = adaptedGoal;
      break;
    }
  }
  if (!isNavigating) navigate();

}

int main(int argc, char **argv) 
{
  
  ros::init(argc, argv, "multi_goal_navigation");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("cb_goal", 1000, addGoal);
  ros::spin();

  navigate();

  return 0;
}
