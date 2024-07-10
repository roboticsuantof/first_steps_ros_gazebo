#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <move_base_msgs/MoveBaseActionResult.h>
// Se define una estructura "Goal" que contiene las coordenadas x,y,yaw (orientacion)
struct Goal {
    double x;
    double y;
    double yaw;
};

// Se declara un array con los puntos a los que debe llegar al robot, 
// estos poseen la estructura anterior (x,y,yaw)
Goal goals[] = {
    {-6, 0, 1.57},
    {-5.66, 1.56, 1.57},
    {-5.66, 4.5, 0},
    {-1, 4.5, -1.57},
    {1, 0, 0},
    {1, 3, 1.57},
    {3, 1, 0},
    {6.5, 4.5, -1.57},
    {6.5, -4.5, -3.14}
};
// Se define la función que publica las metas en el topic move_base_simple/goal
void pubGoal(ros::Publisher &pub, double x, double y, double yaw) {
    geometry_msgs::PoseStamped goal; //Se declara la variable goal,mensaje del tipo PoseStamped
    goal.header.frame_id = "map";  //Se establece el marco de referencia "map"
    goal.header.stamp = ros::Time::now();//Se asigna la hora actual al campo "stamp" 
 // Este mensaje contiene la posición y orientacion, 
 // que posteriormente se convierte a cuaternion                                    
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    

 // Se realiza la conversion a cuaternion 
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);

    goal.pose.orientation.w = q.getW();
    goal.pose.orientation.x = q.getX();
    goal.pose.orientation.y = q.getY();
    goal.pose.orientation.z = q.getZ();

    pub.publish(goal);// Se publica el mensaje
    ROS_INFO("Published goal: x=%f, y=%f, yaw=%f", x, y, yaw);
}
//Se define una variable booleana para comprobar si se llego a la meta
bool ReachedGoal=false; 
// Callback que cambia la variable anterior a true si se recibe un mensaje
// En el topic move_base/result, indicando que se llego a la meta actual
void goalCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
{
    ReachedGoal=true;
}
// Se inicia el nodo ROS, además de crear el suscriptor y publicador necesarios
int main(int argc, char** argv) {
    ros::init(argc, argv, "turtlebot3_pathing");
    ros::NodeHandle nh;

    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
    ros::Subscriber GoalReach_sub = nh.subscribe("move_base/result", 1000, goalCallback);
    
    // Se esperan 2 segundos para que el nodo realice las conexiones a los topics
    ros::Duration(2.0).sleep();

    // Se publican las metas de forma iterativa

    // Se llama a pubGoal para publicar la primera meta del array hasta que se llame al callback, 
    // que avisa que se llego al objetivo, luego se repite el proceso para las siguientes metas
     for (const auto& goal : goals) {
        // Se resetea la variable ReachedGoal a false antes de enviar la nueva meta
        ReachedGoal = false;
        pubGoal(goal_pub, goal.x, goal.y, goal.yaw);

        // Se espera hasta alcanzar la meta actual antes de enviar la siguiente
        while (ros::ok() && !ReachedGoal) {
            ros::spinOnce();
            ros::Duration(0.1).sleep();  
        }
    }
    return 0;
}
