#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <move_base_msgs/MoveBaseActionResult.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

int i, c = 0; // Contadores para los waypoints y para controlar la secuencia

// Matriz de waypoints (x, y, theta)
float waypoints[9][3] = { //
  {-5.65, 0, 1.57},
  {-5.65, 1.3, 1.57},
  {-5.65, 4, 0},
  {-0.73, 4.35, -1.57},
  {0.78, 0.1668, 0},
  {1, 3.3, 1.57},
  {3, 1, 0},
  {6.75, 4.0, -1.57},
  {7, -4.5, -3.14}
};

// Callback que se llama cuando el robot alcanza un objetivo
void goalCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){
  i++; //Incrementa el contador 
}

// Función para publicar un goal
void setGoal(ros::Publisher &pub, int i){
  geometry_msgs::PoseStamped pose; // Se define pose como un mensaje de tipo PoseStamped
  pose.header.frame_id = "map";
  pose.header.stamp = ros::Time::now();
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, waypoints[i][2]); // Transformación de Theta (yaw) a cuaternión (x,y,w,z)
  pose.pose.position.x=waypoints[i][0];
  pose.pose.position.y=waypoints[i][1];
  pose.pose.position.z=0;
  pose.pose.orientation.x=q.x();
  pose.pose.orientation.y=q.y();
  pose.pose.orientation.w=q.w();
  pose.pose.orientation.z=q.z();
  pub.publish(pose); // Se publica el mensaje pose
  ROS_INFO("GOAL PUBLICADO %i", i);
}

int main(int argc, char** argv) {
  // Inicializa el nodo ROS, se configuran las suscripciones y publicadores
  ros::init(argc, argv, "path");
  ros::NodeHandle nh;
  ros::Subscriber goal_sub = nh.subscribe("/move_base/result", 10, goalCallback);
  ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
  ros::Rate loop_rate(10);

  // Se espera a que existan suscriptores en el tópico
  while (goal_pub.getNumSubscribers() == 0) {
    loop_rate.sleep();
  }
  
  // Se publica el primer goal
  setGoal(goal_pub, i);

  // Lazo de contol que publica el nuevo goal, por cada vez que se haya alcanzado un waypoint
  while(ros::ok() && i < 9){
    if(i == c+1){
      c++;
      setGoal(goal_pub, i);
    }
    ros::spinOnce();
  }
  return 0;
}