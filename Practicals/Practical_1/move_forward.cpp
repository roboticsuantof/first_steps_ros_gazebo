#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <unistd.h>

turtlesim::Pose g_pose, g_goal;
bool g_newPose;

void poseCallback(const turtlesim::PoseConstPtr& pose)
{
    g_pose = *pose;
    g_newPose = true;
}

bool hasReachedGoal()
{
    return fabsf(g_pose.x - g_goal.x) < 0.1 && fabsf(g_pose.y - g_goal.y) < 0.1
    && fabsf(g_pose.theta - g_goal.theta) < 0.01;
}
void commandTurtle(ros::Publisher &twist_pub, float linear, float angular)
{
    geometry_msgs::Twist twist;
    twist.linear.x = linear;
    twist.angular.z = angular;
    twist_pub.publish(twist);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_forward");
    ros::NodeHandle nh;
    
    ros::Subscriber pose_sub = nh.subscribe("turtle1/pose", 1,
    poseCallback);
    
    ros::Publisher twist_pub =
    nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
    
    ros::ServiceClient reset = nh.serviceClient<std_srvs::Empty>("reset");

    // Reset the robot position to the initial one
    std_srvs::Empty empty;
    reset.call(empty);
    // Set the goal position ==> Current position + 4 meters in x
    g_newPose = false;

    while(!g_newPose)
    {
        ros::spinOnce();
        usleep(250000);
    }

    g_goal.x = g_pose.x + 4.0;
    g_goal.y = g_pose.y;
    g_goal.theta = g_pose.theta;

    // Move the robot foward 4 meters

    while(ros::ok())
    {
    // Send a command to the robot
    if(hasReachedGoal())
        commandTurtle(twist_pub, 0, 0);
    else
        commandTurtle(twist_pub, 0.75, 0.0);

    // Print current position
    ROS_INFO("Robot position [%f, %f, %f]", g_pose.x, g_pose.y, g_pose.theta);
    // Update ROS
    ros::spinOnce();
    // Sleeps 100ms
    usleep(100000);
    }
    
    return 0;
}