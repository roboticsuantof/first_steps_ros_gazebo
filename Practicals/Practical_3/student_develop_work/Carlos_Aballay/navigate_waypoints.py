#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

class WaypointNavigator:
    def __init__(self):
        rospy.init_node('navigate_waypoints')
        self.waypoints = [
            (-6, -3, 0),#punto1
            (-6, 0, 0),#punto2
            (-6, 1.5, 0),#punto3
            (-5.5, 4.5, 0),#punto4
            (-1, 4.5, 0),#punto5
            (1.5, 0.2, 0),#punto6
            (0.5, 3, 0),#punto7
            (3, 0.3, 0),#punto8
            (6.5, 4.5, 0),#punto9
            (6.5, -4.5, 0)#punto10
        ]
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10) # Publicador para enviar los objetivos de navegación
        rospy.Subscriber('/odom', Odometry, self.odom_callback) # Suscriptor para recibir datos de odometría
        self.current_pose = None
        self.current_index = 0

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose # Callback para actualizar la posición actual del robot

    def distance(self, pos1, pos2):
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2) # Función para calcular la distancia entre dos puntos

    def navigate(self):
        rate = rospy.Rate(1) # Frecuencia de 1 Hz para el bucle de navegación
        
        # Esperar hasta que la odometría esté disponible
        while not rospy.is_shutdown() and self.current_pose is None:
            rospy.loginfo("Esperando datos de odometría...") #lazocerrado
            rate.sleep()

        for waypoint in self.waypoints:
            if rospy.is_shutdown():
                break
            goal = PoseStamped()
            goal.header.frame_id = 'map' #lazocerrado
            goal.pose.position.x = waypoint[0]
            goal.pose.position.y = waypoint[1]
            quaternion = quaternion_from_euler(0, 0, waypoint[2]) #lazocerrado
            goal.pose.orientation.x = quaternion[0]
            goal.pose.orientation.y = quaternion[1]
            goal.pose.orientation.z = quaternion[2]
            goal.pose.orientation.w = quaternion[3]
            self.goal_pub.publish(goal) # Publicar el objetivo de navegación

            rospy.loginfo(f"Navegando a waypoint: {waypoint}")

            # Esperar hasta que el robot alcance el waypoint
            while not rospy.is_shutdown():
                if self.current_pose:
                    current_pos = (self.current_pose.position.x, self.current_pose.position.y) #lazocerrado
                    rospy.loginfo(f"Posición actual: {current_pos}")
                    if self.distance(current_pos, (waypoint[0], waypoint[1])) < 0.5:
                        rospy.loginfo(f"Waypoint {self.current_index + 1} alcanzado")
                        self.current_index += 1
                        rospy.sleep(5)  # Permitir que el robot se estabilice
                        break
                rate.sleep()
        
        rospy.loginfo("Ruta completada") #lazocerrado

if __name__ == '__main__':
    navigator = WaypointNavigator()
    navigator.navigate()

