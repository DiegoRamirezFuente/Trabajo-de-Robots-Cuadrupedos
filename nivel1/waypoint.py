#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from math import atan2, sqrt
from tf.transformations import euler_from_quaternion

# === Variables globales ===
waypoints = [(1, 0), (2.5, 0), (3, -1), (6, -1)]
current_waypoint_index = 0

current_x, current_y, current_theta = 0.0, 0.0, 0.0
current_pitch = 0.0
in_hole = False
cmd_vel_pub = None
gait_pub = None
gait_sent = False

# === Callback de odometría ===
def odom_callback(msg):
    global current_x, current_y, current_theta, current_pitch

    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y

    orientation_q = msg.pose.pose.orientation
    roll, pitch, yaw = euler_from_quaternion([
        orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
    ])

    current_theta = yaw
    current_pitch = pitch

# === Movimiento hacia el waypoint ===
def move_to_waypoint(waypoint):
    global current_x, current_y, current_theta, in_hole, current_pitch, gait_sent

    dx = waypoint[0] - current_x
    dy = waypoint[1] - current_y
    distance = sqrt(dx**2 + dy**2)
    angle_to_goal = atan2(dy, dx)
    angle_diff = angle_to_goal - current_theta

    # Normalizar ángulo
    while angle_diff > 3.14159:
        angle_diff -= 2 * 3.14159
    while angle_diff < -3.14159:
        angle_diff += 2 * 3.14159

    # === Control de inclinación solo a partir del segundo waypoint ===
    pitch_threshold_hole = 0.02
    pitch_threshold_critical = 0.05

    if current_waypoint_index >= 1:
        if abs(current_pitch) > pitch_threshold_critical:
            rospy.logwarn("¡Inclinación peligrosa detectada! Deteniendo robot para evitar vuelco.")
            stop_robot()
            return

        in_hole = abs(current_pitch) > pitch_threshold_hole
    else:
        in_hole = False

    # === Estrategia muro/escalón: a partir del waypoint 2 ===
    is_near_wall = current_waypoint_index >= 2

    if is_near_wall:
        linear_speed = min(0.1 * distance, 0.15)  # velocidad muy reducida
        if not gait_sent:
            rospy.loginfo("Activando gait especial para muro: crawl")
            gait_pub.publish(String("crawl"))
            gait_sent = True
    else:
        linear_speed = min(0.4 * distance, 0.15 if in_hole else 0.35)

    angular_speed = min(0.4 * angle_diff, 0.4)

    cmd_vel = Twist()
    cmd_vel.linear.x = linear_speed
    cmd_vel.angular.z = angular_speed
    cmd_vel_pub.publish(cmd_vel)

# === Bucle de navegación principal ===
def navigate():
    global current_waypoint_index, gait_sent

    rospy.Subscriber('/odom', Odometry, odom_callback)
    rate = rospy.Rate(10)

    start_time = rospy.Time.now()  # Tiempo inicial

    while not rospy.is_shutdown():
        if current_waypoint_index >= len(waypoints):
            end_time = rospy.Time.now()  # Tiempo final
            total_time = (end_time - start_time).to_sec()
            rospy.loginfo("Todos los waypoints alcanzados. Deteniendo robot.")
            rospy.loginfo(f"Tiempo total de recorrido: {total_time:.2f} segundos")
            stop_robot()
            break

        current_waypoint = waypoints[current_waypoint_index]
        move_to_waypoint(current_waypoint)

        dx = current_waypoint[0] - current_x
        dy = current_waypoint[1] - current_y
        distance = sqrt(dx**2 + dy**2)

        if distance < 0.15:
            rospy.loginfo(f"Waypoint {current_waypoint} alcanzado")

            if current_waypoint_index == 0 and not gait_sent:
                rospy.loginfo("Activando gait: crawl")
                gait_pub.publish(String("crawl"))
                gait_sent = True

            current_waypoint_index += 1

        rate.sleep()

# === Función para detener el robot ===
def stop_robot():
    cmd_vel = Twist()
    cmd_vel.linear.x = 0.0
    cmd_vel.angular.z = 0.0
    cmd_vel_pub.publish(cmd_vel)

# === Main ===
if __name__ == '__main__':
    try:
        rospy.init_node('waypoint_navigation', anonymous=True)
        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        gait_pub = rospy.Publisher('/gait_command', String, queue_size=10)
        navigate()
    except rospy.ROSInterruptException:
        pass
