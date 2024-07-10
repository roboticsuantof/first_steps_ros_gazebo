#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <vector>

// Declaración del publicador para enviar los objetivos de navegación
ros::Publisher goal_pub;

// Variable para almacenar la posición actual del robot
geometry_msgs::Pose current_pose;

// Callback para la odometría: actualiza la posición actual del robot
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_pose = msg->pose.pose;
}

// Función para enviar un objetivo de navegación al robot
void sendGoal(double x, double y, double theta) {
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();

    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = 0.0;

    tf::Quaternion q;
    q.setRPY(0, 0, theta);
    tf::quaternionTFToMsg(q, goal.pose.orientation);

    // Publicar el objetivo de navegación
    goal_pub.publish(goal);
    ROS_INFO("New goal sent: x = %f, y = %f, theta = %f", x, y, theta);
}

// Función para comprobar si se ha alcanzado el objetivo de navegación
bool hasReachedGoal(double goal_x, double goal_y, double threshold = 0.3) {
    double distance = sqrt(pow(current_pose.position.x - goal_x, 2) + pow(current_pose.position.y - goal_y, 2));
    return distance < threshold;
}

int main(int argc, char** argv) {
    // Inicializar el nodo ROS
    ros::init(argc, argv, "move_to_waypoints");
    ros::NodeHandle nh;

    // Crear el publicador para enviar objetivos de navegación
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

    // Suscribirse a la odometría para obtener la posición actual del robot
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);

    // Esperar 2 segundos para asegurarse de que todo esté configurado
    ros::Duration(2.0).sleep();

    // Definir los waypoints a visitar
    std::vector<std::vector<double>> waypoints = {
        {-7.0, -3.0, 0.0}, // coordenada 0 , Inicio
        {-6.0, -3.0, 0.0}, // coordenada 1
        {-6.0, 0.0, 0.0},  // coordenada 2
        {-6.0, 2.0, 0.0},  // coordenada 3
        {-6.0, 4.0, 0.0},  // coordenada 4
        {-1.0, 4.0, 0.0},  // coordenada 5
        {1.0, 0.43, 0.0},  // coordenada 6
        {1.0, 3.0, 0.0},   // coordenada 7
        {4.0, 1.0, 0.0},   // coordenada 8
        {6.65, 4.0, 0.0},  // coordenada 9
        {7.0, -4.0, 0.0}   // coordenada 10 , Final 
    };

    // Crear un objeto Rate para controlar el ciclo de espera
    ros::Rate loop_rate(10);

    // Iterar a través de los waypoints
    for (const auto& waypoint : waypoints) {
        sendGoal(waypoint[0], waypoint[1], waypoint[2]);

        // Esperar hasta que se alcance el waypoint actual
        while (ros::ok() && !hasReachedGoal(waypoint[0], waypoint[1])) {
            ros::spinOnce();
            loop_rate.sleep();
        }

        // Usar ANSI escape code para color amarillo al llegar al waypoint
        ROS_INFO("\033[1;33mReached waypoint: x = %f, y = %f\033[0m", waypoint[0], waypoint[1]);
        ros::Duration(1.0).sleep();
    }

    // Indicar que se han alcanzado todos los waypoints y detener el robot
    ROS_INFO("All waypoints reached. Stopping the robot.");
    return 0;
}
