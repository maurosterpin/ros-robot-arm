#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <robot_remote/RobotTaskAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>


class TaskServer
{
private:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<robot_remote::RobotTaskAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to publish result
  robot_remote::RobotTaskResult result_;
  std::vector<double> arm_goal_;
  std::vector<double> gripper_goal_;
  moveit::planning_interface::MoveGroupInterface arm_move_group_;
  moveit::planning_interface::MoveGroupInterface gripper_move_group_;

public:

  // Constructor
  // function that inizialize the RobotTaskAction class and creates 
  // a Simple Action Server from the library actionlib
  TaskServer(std::string name) :
    as_(nh_, name, boost::bind(&TaskServer::execute_cb, this, _1), false)
    , action_name_(name)
    , arm_move_group_("robot_arm")
    , gripper_move_group_("robot_hand")
  {
    as_.start();
  }

  void execute_cb(const robot_remote::RobotTaskGoalConstPtr &goal)
  {
    bool success = true;

    // start executing the action
    // based on the goal id received, send a different goal 
    // to the robot
    if (goal->task_number == 0)
    {
      arm_goal_ = {0.0, 0.0, 0.0};
      gripper_goal_ = {-0.7, 0.7};
      arm_move_group_.setJointValueTarget(arm_goal_);
      gripper_move_group_.setJointValueTarget(gripper_goal_);
      arm_move_group_.move();
      gripper_move_group_.move();
      arm_move_group_.stop();
      gripper_move_group_.stop();
    }
    else if (goal->task_number == 1)
    {
      arm_goal_ = {-1.14, -0.6, -0.08};
      arm_move_group_.setJointValueTarget(arm_goal_);
      arm_move_group_.move();

      gripper_goal_ = {-0.2, 0.2};
      gripper_move_group_.setJointValueTarget(gripper_goal_);
      gripper_move_group_.move();

      arm_goal_ = {0.0, 0.0, 0.0};
      arm_move_group_.setJointValueTarget(arm_goal_);
      arm_move_group_.move();

      gripper_goal_ = {-0.7, 0.7};
      gripper_move_group_.setJointValueTarget(gripper_goal_);
      gripper_move_group_.move();
    }
    else if (goal->task_number == 2)
    {
      arm_goal_ = {-1.14, -0.6, -0.08};
      arm_move_group_.setJointValueTarget(arm_goal_);
      arm_move_group_.move();

      gripper_goal_ = {-0.2, 0.2};
      gripper_move_group_.setJointValueTarget(gripper_goal_);
      gripper_move_group_.move();

      arm_goal_ = {0.0, 0.0, 0.0};
      arm_move_group_.setJointValueTarget(arm_goal_);
      arm_move_group_.move();

      gripper_goal_ = {-0.7, 0.7};
      gripper_move_group_.setJointValueTarget(gripper_goal_);
      gripper_move_group_.move();
    }
    else
    {
      ROS_ERROR("Invalid goal");
      return;
    }

    // Sends a goal to the moveit API
    // arm_move_group_.setJointValueTarget(arm_goal_);
    // gripper_move_group_.setJointValueTarget(gripper_goal_);

    // blocking functions below, will return after the execution
    // arm_move_group_.move();
    // gripper_move_group_.move();

    // Make sure that no residual movement remains
    // arm_move_group_.stop();
    // gripper_move_group_.stop();

    // check that preempt has not been requested by the client
    if (as_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      success = false;
    }

    // check if the goal request has been executed correctly
    if(success)
    {
      result_.success = true;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  // Inizialize a ROS node called task_server
  ros::init(argc, argv, "task_server");
  TaskServer server("task_server");

  // keeps the node up and running
  ros::spin();
  return 0;
}