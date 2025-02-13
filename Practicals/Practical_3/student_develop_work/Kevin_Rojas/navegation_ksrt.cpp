#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <cmath>

// Definir un SimpleActionClient para enviar los objetivos a move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Función para verificar si el robot está dentro del radio de la zona objetivo
bool isWithinGoalRadius(const tf::TransformListener& listener, const move_base_msgs::MoveBaseGoal& goal, double goal_radius) {
    tf::StampedTransform transform;
    try {
        // Esperar a que la transformación esté disponible
        listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
        // Obtener la transformación más reciente entre "map" y "base_link"
        listener.lookupTransform("map", "base_link", ros::Time(0), transform);
    } catch (tf::TransformException &ex) {
        ROS_WARN("%s", ex.what());  // Advertir si hay un problema al obtener la transformación
        return false;
    }

    // Calcular la distancia entre la posición actual del robot y el objetivo
    double dx = transform.getOrigin().x() - goal.target_pose.pose.position.x;
    double dy = transform.getOrigin().y() - goal.target_pose.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);  // Distancia euclidiana

    // Verificar si la distancia está dentro del radio de la zona objetivo
    return distance <= goal_radius;
}

int main(int argc, char** argv) {
    // Inicializar el nodo de ROS
    ros::init(argc, argv, "turtlebot3_navigation_cpp");

    // Crear un cliente de acción para interactuar con move_base
    MoveBaseClient ac("move_base", true);

    // Esperar a que el servidor de acciones se inicie
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Esperando a que el servidor de acciones de move_base se inicie");
    }

    // Definir los waypoints
    std::vector<std::vector<double>> waypoints = {
        {-6.06, -2.97, 0.0},
        {-5.62, 0.01, 0.0},
        {-6.012, 2.01, 0.0},
        {-5.98, 4.02, 0.0},
        {-1.45, 3.99, 0.0},
        {1.60, 0.31, 0.0},
        {1.05, 3.59, 0.0},
        {3.23, 1.02, 0.0},
        {6.77, 4.45, 0.0},
        {6.59, -4.79, 0.0}
    };

    // Radio de la zona objetivo
    double goal_radius = 0.5;

    // Crear un transform listener para obtener la posición actual del robot
    tf::TransformListener listener;

    // Bucle a través de cada waypoint
    for (int i = 0; i < waypoints.size(); ++i) {
        move_base_msgs::MoveBaseGoal goal;

        // Configurar los parámetros del objetivo
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = waypoints[i][0];
        goal.target_pose.pose.position.y = waypoints[i][1];
        goal.target_pose.pose.orientation.w = 1.0;  // Suponiendo orientación θ = 0

        ROS_INFO("Enviando objetivo %d: (%f, %f)", i + 1, waypoints[i][0], waypoints[i][1]);
        ac.sendGoal(goal);  // Enviar el objetivo al action server

        // Esperar a que el robot alcance la zona objetivo o que el objetivo se complete
        while (ros::ok()) {
            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("El robot se movió al objetivo %d", i + 1);
                break;
            }

            if (isWithinGoalRadius(listener, goal, goal_radius)) {
                ROS_INFO("El robot ha alcanzado la zona del objetivo %d", i + 1);
                break;
            }

            ros::Duration(0.1).sleep();  // Esperar 0.1 segundos antes de verificar nuevamente
        }

        if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED && !isWithinGoalRadius(listener, goal, goal_radius)) {
            ROS_WARN("El robot no pudo moverse al objetivo %d por alguna razón", i + 1);
        }
    }

    return 0;  // Finalizar el programa
}

