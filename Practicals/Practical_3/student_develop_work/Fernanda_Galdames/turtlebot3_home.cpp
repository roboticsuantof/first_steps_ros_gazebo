#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <vector>
#include <cmath>

// Constantes
const std::string ODOM_TOPIC = "/odom";
const std::string GOAL_TOPIC = "/move_base_simple/goal";
const double REACHED_THRESHOLD = 0.3;
const double LOOP_RATE = 10.0;
const double INITIAL_SLEEP_DURATION = 2.0;
const double WAYPOINT_REACHED_SLEEP_DURATION = 1.0;

ros::Publisher goal_pub;
geometry_msgs::Pose current_pose;

// Callback para actualizar la posición actual del robot
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_pose = msg->pose.pose;
}

// Publicar un nuevo objetivo
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

    goal_pub.publish(goal);
    ROS_INFO("New goal sent: x = %f, y = %f, theta = %f", x, y, theta);
}

// Verificar si se ha alcanzado el objetivo
bool hasReachedGoal(double goal_x, double goal_y, double threshold = REACHED_THRESHOLD) {
    double distance = std::sqrt(std::pow(current_pose.position.x - goal_x, 2) + std::pow(current_pose.position.y - goal_y, 2));
    return distance < threshold;
}

int main(int argc, char** argv) {
    // Inicialización de ROS
    ros::init(argc, argv, "move_to_waypoints");
    ros::NodeHandle nh;

    // Publicador de objetivos
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>(GOAL_TOPIC, 10);
    // Suscriptor de la odometría
    ros::Subscriber odom_sub = nh.subscribe(ODOM_TOPIC, 10, odomCallback);

    // Esperar un tiempo para asegurar la inicialización completa
    ros::Duration(INITIAL_SLEEP_DURATION).sleep();

    // Lista de waypoints
    std::vector<std::vector<double>> waypoints = {
        {-7.0, -3.0, 0.0},
        {-6.0, -3.0, 0.0},
        {-6.0, 0.0, 0.0},
        {-6.0, 2.0, 0.0},
        {-6.0, 4.0, 0.0},
        {-1.0, 4.0, 0.0},
        {1.0, 0.43, 0.0},
        {1.0, 3.0, 0.0},
        {4.0, 1.0, 0.0},
        {6.65, 4.0, 0.0},
        {7.0, -4.0, 0.0}  // Nueva coordenada agregada
    };

    // Establecer la frecuencia de ejecución del bucle
    ros::Rate loop_rate(LOOP_RATE);

    // Iterar sobre cada waypoint
    for (const auto& waypoint : waypoints) {
        // Enviar el objetivo actual
        sendGoal(waypoint[0], waypoint[1], waypoint[2]);

        // Esperar hasta que se alcance el objetivo
        while (ros::ok() && !hasReachedGoal(waypoint[0], waypoint[1])) {
            ros::spinOnce();
            loop_rate.sleep();
        }

        // Usar ANSI escape code para color amarillo
        ROS_INFO("\033[1;33mReached waypoint: x = %f, y = %f\033[0m", waypoint[0], waypoint[1]);
        // Pausa después de alcanzar un waypoint
        ros::Duration(WAYPOINT_REACHED_SLEEP_DURATION).sleep();
    }

    // Informar que se han alcanzado todos los waypoints
    ROS_INFO("All waypoints reached. Stopping the robot.");
    return 0;
}