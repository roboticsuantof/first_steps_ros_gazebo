#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Definir un SimpleActionClient para enviar los objetivos a move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "turtlebot3_navigation_cpp");

  // Crear un cliente de acción para interactuar con move_base
  MoveBaseClient ac("move_base", true);

  // Esperar a que el servidor de acciones se inicie
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Esperando a que el servidor de acciones de move_base se inicie");
  }

  // Definir los waypoints
  std::vector<std::vector<double>> waypoints = {
    {-5.82, -3.27, 0.0},
    {-6.26, 0.40, 0.0},
    {-5.65, 1.67, 0.0},
    {-5.69, 4.60, 0.0},
    {-0.97, 4.42, 0.0},
    {1.04, 0.36, 0.0},
    {0.93, 2.14, 0.0},
    {3.41, 0.93, 0.0},
    {6.71, 4.32, 0.0},
    {6.33, -4.45, 0.0}
  };

  for(int i = 0; i < waypoints.size(); ++i) {
    move_base_msgs::MoveBaseGoal goal;

    // Configurar los parámetros 
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

 
    goal.target_pose.pose.position.x = waypoints[i][0];
    goal.target_pose.pose.position.y = waypoints[i][1];
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Enviando objetivo %d: (%f, %f)", i+1, waypoints[i][0], waypoints[i][1]);
    ac.sendGoal(goal);

    
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("El robot se movió al objetivo %d", i+1);
    else
      ROS_INFO("El robot no pudo moverse al objetivo %d por alguna razón", i+1);
  }

  return 0;
}
