#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

move_base_msgs::MoveBaseGoal goalList [50]; // setup goal list of up to 50 goals

void clearGoalList () { // sets all goals to 0,0,0
  
  for (int i = 0; i < 50; ++i) {
    goalList[i].target_pose.pose.position.x = 0;
    goalList[i].target_pose.pose.position.y = 0;
    goalList[i].target_pose.pose.position.z = 0;
  }
  ROS_INFO("cleared goal list");

}

void navigate(const std_msgs::BoolConstPtr& nav) { // begins navigation

  if (!nav) { // if given a false, cancel navigation and clear the goal list
    clearGoalList();
    return;
  }

  MoveBaseClient ac("move_base", true);
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server");
  } 
  ROS_INFO("Executing goals");
  for (int i = 0; i < sizeof(goalList); i++) { // run through the goal list, navigating to each until the end where the jackal navigates home
    if (goalList[i].target_pose.pose.position.x == 0 && goalList[i].target_pose.pose.position.y == 0) break;
    ROS_INFO("Navigating to goal");
    ac.sendGoal(goalList[i]);
    ac.waitForResult(ros::Duration(0,0));

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("The jackal reached the goal");
    } else {
      ROS_INFO("The jackal failed to reach the goal");
    }
  }
    
  move_base_msgs::MoveBaseGoal homeGoal; // set up the home goal at 0,0,0 for now
  homeGoal.target_pose.header.frame_id = "map";
  homeGoal.target_pose.header.stamp = ros::Time::now();
  homeGoal.target_pose.pose.position.x = 0.0;
  homeGoal.target_pose.pose.position.y = 0.0;
  homeGoal.target_pose.pose.position.z = 0.0;
  homeGoal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Navigating home");
  ac.sendGoal(homeGoal);

  ac.waitForResult(ros::Duration(0,0));

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("The jackal reached the docking station");
  } else {
    ROS_INFO("The jackal failed to reach the docking station");
  }

  clearGoalList();

}


void addGoal(const geometry_msgs::PoseStampedConstPtr& goal) { // adds goal to the list from the cb_goal topic
  
  move_base_msgs::MoveBaseGoal adaptedGoal;
  adaptedGoal.target_pose.header.frame_id = "map";
  adaptedGoal.target_pose.header.stamp = ros::Time::now();

  adaptedGoal.target_pose.pose.position.x = goal->pose.position.x;
  adaptedGoal.target_pose.pose.position.y = goal->pose.position.y;
  adaptedGoal.target_pose.pose.position.z = goal->pose.position.z;
  adaptedGoal.target_pose.pose.orientation.w = goal->pose.orientation.w;
  
  for (int i = 0; i < sizeof(goalList); i++) {
    if (goalList[i].target_pose.pose.position.x == 0 && goalList[i].target_pose.pose.position.y == 0) {
      goalList[i] = adaptedGoal;
      break;
    }
  }

  ROS_INFO("added goal");  
}



int main(int argc, char **argv) 
{
  
  ros::init(argc, argv, "multi_goal_navigation");
  ros::NodeHandle nh;
  nh.setParam("move_base/actionlib_client_sub_queue_size", 30);
  nh.setParam("move_base/actionlib_client_pub_queue_size", 30);
  clearGoalList();
  ros::Subscriber sub = nh.subscribe("cb_goal", 1000, addGoal);    // subscribes to "cb_goal" where our jackal goals are published
  ros::Subscriber sub2 = nh.subscribe("navigate", 1000, navigate); // subscribes to "navigate" where the command to start the substation inspection is sent
  ros::spin();
 
  return 0;
}
