#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <vector>

// Declaración de publicadores y la pose actual del robot
ros::Publisher goal_pub;  // Publicador para enviar objetivos de pose
ros::Publisher vel_pub;   // Publicador para enviar comandos de velocidad
geometry_msgs::Pose current_pose;  // Pose actual del robot

// Callback para actualizar la pose actual del robot a partir de la odometría
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_pose = msg->pose.pose;
}

// Función para enviar un objetivo de pose al robot
void sendGoal(double x, double y, double theta) {
    geometry_msgs::PoseStamped goal;  // Mensaje de pose objetivo
    goal.header.frame_id = "map";     // Frame de referencia
    goal.header.stamp = ros::Time::now();  // Marca de tiempo actual

    // Establecer las coordenadas del objetivo
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = 0.0;

    // Convertir el ángulo theta a un quaternion y asignarlo a la orientación del objetivo
    tf::Quaternion q;
    q.setRPY(0, 0, theta);
    tf::quaternionTFToMsg(q, goal.pose.orientation);

    // Publicar el objetivo
    goal_pub.publish(goal);
    ROS_INFO("New goal sent: x = %f, y = %f, theta = %f", x, y, theta);
}

// Función para verificar si el robot ha alcanzado el objetivo
bool hasReachedGoal(double goal_x, double goal_y, double threshold = 0.3) {
    // Calcular la distancia entre la pose actual y el objetivo
    double distance = sqrt(pow(current_pose.position.x - goal_x, 2) + pow(current_pose.position.y - goal_y, 2));
    return distance < threshold;  // Verificar si la distancia es menor que el umbral
}

int main(int argc, char** argv) {
    // Inicializar el nodo ROS
    ros::init(argc, argv, "move_to_waypoints");
    ros::NodeHandle nh;

    // Inicializar los publicadores
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Suscribirse al tema de odometría
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);

    // Esperar 2 segundos para asegurarse de que todo esté configurado correctamente
    ros::Duration(2.0).sleep();

    // Definir una lista de puntos de referencia (waypoints) a los que el robot debe moverse
    std::vector<std::vector<double>> waypoints = {
        {-6.0, -3.0, 0.0}, // 1 punto 
        {-6.0, 0.0, 0.0},  // 2 punto
        {-6.0, 2.0, 0.0},  // 3 puntp
        {-6.0, 4.0, 0.0},  // 4 punto
        {-1.0, 4.0, 0.0},  // 5 punto
        {1.0, 0.43, 0.0},  // 6 punto
        {1.0, 3.0, 0.0},   // 7 punto
        {4.0, 1.0, 0.0},   // 8 punto
        {6.65, 4.0, 0.0},  // 9 punto
        {6.65, -3.5, 0.0}  // 10 punto
    };

    ros::Rate loop_rate(10);  // Frecuencia de ejecución del bucle principal

    // Iterar a través de cada punto de referencia
    for (const auto& waypoint : waypoints) {
        sendGoal(waypoint[0], waypoint[1], waypoint[2]);  // Enviar objetivo

        // Esperar hasta que el robot alcance el objetivo
        while (ros::ok() && !hasReachedGoal(waypoint[0], waypoint[1])) {
            ros::spinOnce();  // Procesar callbacks
            loop_rate.sleep();  // Dormir para mantener la frecuencia de bucle
        }

        // Usar ANSI escape code para color verde
        ROS_INFO("\033[1;32mReached waypoint: x = %f, y = %f\033[0m", waypoint[0], waypoint[1]);
        ros::Duration(1.0).sleep();  // Esperar un segundo antes de proceder al siguiente punto de referencia
    }

    // Detener el robot enviando un comando de velocidad cero
    geometry_msgs::Twist stop_vel;
    stop_vel.linear.x = 0.0;
    stop_vel.linear.y = 0.0;
    stop_vel.linear.z = 0.0;
    stop_vel.angular.x = 0.0;
    stop_vel.angular.y = 0.0;
    stop_vel.angular.z = 0.0;

    vel_pub.publish(stop_vel);  // Publicar el mensaje de velocidad cero
    ROS_INFO("All waypoints reached. Stopping the robot.");

    return 0;  // Finalizar el programa
}