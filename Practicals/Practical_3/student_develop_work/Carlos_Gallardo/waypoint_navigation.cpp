#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

// Define a SimpleActionClient to send goals to move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Function to check if the robot is within a specified tolerance of the goal
bool isWithinTolerance(const geometry_msgs::Pose& goal_pose, const geometry_msgs::Pose& current_pose, double tolerance) {
  double dx = goal_pose.position.x - current_pose.position.x;
  double dy = goal_pose.position.y - current_pose.position.y;
  return std::sqrt(dx * dx + dy * dy) <= tolerance;
}

// Function to clear the costmaps
void clearCostmaps() {
  ros::NodeHandle nh;
  ros::ServiceClient clear_costmaps_client = nh.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");
  std_srvs::Empty srv;
  clear_costmaps_client.call(srv);
}

// Function to get the current position of the robot
geometry_msgs::Pose getCurrentPose() {
  nav_msgs::Odometry::ConstPtr msg = ros::topic::waitForMessage<nav_msgs::Odometry>("odom", ros::Duration(1.0));
  if (msg != nullptr) {
    return msg->pose.pose;
  } else {
    ROS_WARN("Failed to get the current position of the robot.");
    return geometry_msgs::Pose(); // Return an empty pose in case of error
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "waypoint_navigation_cpp");

  // Create an action client to interact with move_base
  MoveBaseClient ac("move_base", true);

  // Wait for the action server to start
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to start");
  }

  // Define the waypoints
  std::vector<std::vector<double>> waypoints = {
    {-5.8, -3.3, 0.0},
    {-6.3, 0.4, 0.0},
    {-5.7, 1.7, 0.0},
    {-5.7, 4.6, 0.0},
    {-1.0, 4.4, 0.0},
    {1.0, 0.4, 0.0},
    {0.9, 2.1, 0.0},
    {3.4, 0.9, 0.0},
    {6.7, 4.3, 0.0},
    {6.3, -4.5, 0.0}
  };

  const double tolerance = 0.2; // Tolerance of 20 cm
  const double time_limit = 300.0; // Time limit of 300 seconds per goal
  const int max_retries = 5; // Maximum number of retries

  for(int i = 0; i < waypoints.size(); ++i) {
    move_base_msgs::MoveBaseGoal goal;

    // Set the goal parameters
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = waypoints[i][0];
    goal.target_pose.pose.position.y = waypoints[i][1];

    // Set a valid orientation (0 degrees in this case)
    tf::Quaternion q;
    q.setRPY(0, 0, waypoints[i][2]); // Use the Z angle given in the waypoints
    goal.target_pose.pose.orientation.x = q.x();
    goal.target_pose.pose.orientation.y = q.y();
    goal.target_pose.pose.orientation.z = q.z();
    goal.target_pose.pose.orientation.w = q.w();

    ROS_INFO("Sending goal %d: (%f, %f)", i+1, waypoints[i][0], waypoints[i][1]);
    ac.sendGoal(goal);

    ros::Time start_time = ros::Time::now();
    bool reached_goal = false;
    int retries = 0;

    while(!reached_goal && (ros::Time::now() - start_time).toSec() < time_limit && retries < max_retries) {
      ac.waitForResult(ros::Duration(1.0)); // Check every second

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        // Get the current position of the robot
        geometry_msgs::Pose current_pose = getCurrentPose();

        // Check if it is within tolerance
        reached_goal = isWithinTolerance(goal.target_pose.pose, current_pose, tolerance);
      } else if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED) {
        ROS_WARN("Goal aborted, trying to clear costmaps and retry.");
        clearCostmaps();
        ac.sendGoal(goal);
        retries++;
      }

      if(reached_goal) {
        ROS_INFO("The robot moved to goal %d", i+1);
      }
    }

    if(!reached_goal) {
      ROS_WARN("The robot could not move to goal %d within the time limit of %f seconds or after %d retries", i+1, time_limit, retries);
    }
  }

  return 0;
}

