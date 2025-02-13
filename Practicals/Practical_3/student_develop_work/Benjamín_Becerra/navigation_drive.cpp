
//Benjamín Becerra Páez - 20.845.444-7
//Pŕactica 3 - Control Estabilizante

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

// Define el alias para el cliente de acciones del método move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Método para comprobar si el objetivo ha sido alcanzado
bool isGoalReached(const tf::TransformListener& listener, const move_base_msgs::MoveBaseGoal& goal, double tolerance) {
  tf::StampedTransform transform;
  try {
    // Obtiene la transformada entre el mapa y el robot
    listener.lookupTransform("map", "base_link", ros::Time(0), transform);
    // Calcula la distancia entre la posición actual del robot y el objetivo
    double dx = goal.target_pose.pose.position.x - transform.getOrigin().x();
    double dy = goal.target_pose.pose.position.y - transform.getOrigin().y();
    double distance = sqrt(dx*dx + dy*dy);
    // Verifica si la distancia es menor o igual a la tolerancia
    return distance <= tolerance;
  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }
}

int main(int argc, char** argv) {
  // Se inicializa el nodo ROS
  ros::init(argc, argv, "turtlebot3_navigation_cpp");
  ros::NodeHandle nh;

  // Crea un cliente de acciones para el método move_base
  MoveBaseClient ac("move_base", true);
  tf::TransformListener listener;

  // Espera a que el servidor de acciones del método move_base se inicie
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Esperando a que el servidor de acciones de move_base se inicie");
  }

  // Define los waypoints a los que el robot debe moverse
  std::vector<std::vector<double>> waypoints = {
    {-5.9, -2.9, 0.0},
    {-5.9, -0.0, 0.0},
    {-5.8, 1.8, 0.0},
    {-5.6, 4.1, 0.0},
    {-1.0, 3.9, -0.0},
    {1.0, 0.3, -0.0},
    {1.0, 3.6, -0.0},
    {3.6, 0.8, -0.0},
    {6.7, 3.8, -0.0},
    {6.1, -4.3, -0.0}
  };

  double tolerance = 0.5; // Tolerancia de 0.5 metros

  // Se itera sobre cada waypoint
  for (int i = 0; i < waypoints.size(); ++i) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = waypoints[i][0];
    goal.target_pose.pose.position.y = waypoints[i][1];
    goal.target_pose.pose.orientation.w = 1.0;

    // Envia el objetivo al cliente de acciones
    ROS_INFO("Enviando objetivo %d: (%f, %f)", i + 1, waypoints[i][0], waypoints[i][1]);
    ac.sendGoal(goal);

    ros::Rate rate(10); // Frecuencia de 10 Hz
    while (ros::ok()) {
      // Verifica si el objetivo ha sido alcanzado o si el cliente de acciones ha finalizado exitosamente
      if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED || isGoalReached(listener, goal, tolerance)) {
        ROS_INFO("El robot se movió al objetivo %d", i + 1);
        break;
      }
      // Verificar si el cliente de acciones ha fallado
      if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED) {
        ROS_INFO("El robot no pudo moverse al objetivo %d por alguna razón", i + 1);
        break;
      }
      rate.sleep(); // Esperar hasta la próxima iteración
    }
  }

  return 0;
}
