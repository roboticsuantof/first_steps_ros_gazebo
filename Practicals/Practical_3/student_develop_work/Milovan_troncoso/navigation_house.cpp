
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <utility>

// Definimos el tipo de cliente de acción para move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class MoveRobot {
public:
    MoveRobot() : ac("move_base", true) {
        // Esperamos a que el servidor de acciones de move_base esté listo
        while (!ac.waitForServer(ros::Duration(5.0))) {
            ROS_INFO("Waiting for the move_base action server to come up");
        }
    }

    void moveToGoal(double x, double y) {
        // Creamos un objetivo para move_base
        move_base_msgs::MoveBaseGoal goal;

        // Configuramos el marco de referencia y el tiempo para el objetivo
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        // Definimos la posición objetivo (x, y) y una orientación fija
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        goal.target_pose.pose.orientation.w = 1.0;

        // Enviamos el objetivo al servidor de acciones
        ROS_INFO("Sending goal: x=%f, y=%f", x, y);
        ac.sendGoal(goal);

        // Esperamos a que el robot alcance el objetivo
        ac.waitForResult();

        // Verificamos el estado del objetivo y mostramos el resultado
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Hooray, the base moved to goal: x=%f, y=%f", x, y);
        } else {
            ROS_WARN("The base failed to move to goal: x=%f, y=%f for some reason", x, y);
        }
    }

    void moveToWaypoints(const std::vector<std::pair<double, double>>& waypoints) {
        // Verificamos que haya waypoints proporcionados
        if (waypoints.empty()) {
            ROS_WARN("No waypoints provided");
            return;
        }

        // Movemos el robot a cada waypoint excepto el último
        for (size_t i = 0; i < waypoints.size() - 1; ++i) {
            moveToGoal(waypoints[i].first, waypoints[i].second);
        }

        // Movemos el robot al último waypoint
        moveToGoal(waypoints.back().first, waypoints.back().second);
    }

private:
    MoveBaseClient ac; // Cliente de acción para interactuar con move_base
};

int main(int argc, char** argv) {
    // Inicializamos el nodo ROS
    ros::init(argc, argv, "move_to_waypoints");
    ros::NodeHandle nh;

    // Creamos una instancia de MoveRobot
    MoveRobot mover;

    // Definimos una lista de waypoints (pares de coordenadas x, y)
    std::vector<std::pair<double, double>> waypoints = {
        {-1.0 , 0.53},
        {2.14, 0.35},
        {6.36, 0.42},
        {6.43, -4.47},
        {6.40, 4.04},
        {0.98, 3.03},
        {1.08, 0.39},
        {-2.81, 4.09},
        {-6.31, 2.98},
        {-6.5, -1.95}
    };

    // Movemos el robot a través de los waypoints definidos
    mover.moveToWaypoints(waypoints);

    return 0;
}






