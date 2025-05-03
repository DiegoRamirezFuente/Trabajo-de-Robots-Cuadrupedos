#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from math import atan2, sqrt
from tf.transformations import euler_from_quaternion

# === Variables globales ===
waypoints = [(1.7, 0), (1.7, 0.3), (3, 0.3)]
current_waypoint_index = 0

current_x, current_y, current_theta = 0.0, 0.0, 0.0
current_pitch = 0.0
current_roll = 0.0
cmd_vel_pub = None
gait_pub = None
gait_sent = False
last_linear_speed = 0.0
real_speed = 0.0

# Estado de atasco
stuck = False
stuck_distance = None
recovery_speed = 0.4  # Inicial para impulso

# Constantes de control
ROLL_CRITICAL = 0.2
MAX_SPEED = 0.8
MIN_SPEED = 0.1
PITCH_GAIN = 2.5
WAYPOINT_TOLERANCE = 0.05
STUCK_ADVANCE_REQUIRED = 0.2
STUCK_TIMEOUT = 2.0

# Variables para velocidad real
last_position_check_time = 0
last_position_x = 0
last_position_y = 0
prev_x = 0.0
prev_y = 0.0
prev_time = 0.0

# === Callback de odometría ===
def odom_callback(msg):
    global current_x, current_y, current_theta, current_pitch, current_roll
    global prev_x, prev_y, prev_time, real_speed

    current_time = rospy.get_time()
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y

    dt = current_time - prev_time if prev_time > 0 else 0.1
    dx = current_x - prev_x
    dy = current_y - prev_y
    real_speed = sqrt(dx**2 + dy**2) / dt

    prev_x = current_x
    prev_y = current_y
    prev_time = current_time

    orientation_q = msg.pose.pose.orientation
    roll, pitch, yaw = euler_from_quaternion([
        orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
    ])

    current_theta = yaw
    current_pitch = pitch
    current_roll = roll

# === Función de distancia entre dos puntos ===
def distance_between(x1, y1, x2, y2):
    return sqrt((x1 - x2)**2 + (y1 - y2)**2)

# === Movimiento hacia el waypoint ===
def move_to_waypoint(waypoint):
    global last_linear_speed, stuck, stuck_distance, recovery_speed

    if abs(current_roll) > ROLL_CRITICAL:
        rospy.logerr("¡Inclinación lateral crítica! Deteniendo robot para evitar vuelco.")
        stop_robot()
        rospy.sleep(2.0)
        return

    dx = waypoint[0] - current_x
    dy = waypoint[1] - current_y
    distance = sqrt(dx**2 + dy**2)
    angle_to_goal = atan2(dy, dx)
    angle_diff = angle_to_goal - current_theta

    while angle_diff > 3.14159:
        angle_diff -= 2 * 3.14159
    while angle_diff < -3.14159:
        angle_diff += 2 * 3.14159

    if stuck:
        remaining = distance_between(current_x, current_y, stuck_distance[0], stuck_distance[1])
        if distance < WAYPOINT_TOLERANCE or remaining > STUCK_ADVANCE_REQUIRED:
            rospy.loginfo("Robot ha salido del atasco.")
            stuck = False
            recovery_speed = 0.4
            linear_speed = max(MIN_SPEED, min(MAX_SPEED, 0.3 + PITCH_GAIN * current_pitch))
        else:
            recovery_speed = min(recovery_speed + 0.05, MAX_SPEED)
            linear_speed = recovery_speed
    else:
        linear_speed = max(MIN_SPEED, min(MAX_SPEED, 0.3 + PITCH_GAIN * current_pitch))

    last_linear_speed = linear_speed

    if abs(angle_diff) > 0.2:
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.3 * angle_diff
    else:
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_speed
        cmd_vel.angular.z = 0.4 * angle_diff

    cmd_vel_pub.publish(cmd_vel)

# === Detección de atasco ===
def check_stuck():
    global last_position_check_time, last_position_x, last_position_y, stuck, stuck_distance

    now = rospy.get_time()
    if now - last_position_check_time >= STUCK_TIMEOUT:
        movement = distance_between(current_x, current_y, last_position_x, last_position_y)
        if movement < 0.15:
            if not stuck:
                rospy.logwarn("Posible atasco detectado. Iniciando impulso de recuperación.")
                stuck = True
                stuck_distance = (current_x, current_y)
        last_position_x = current_x
        last_position_y = current_y
        last_position_check_time = now

# === Bucle de navegación principal ===
def navigate():
    global current_waypoint_index, gait_sent

    rospy.Subscriber('/odom', Odometry, odom_callback)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if current_waypoint_index >= len(waypoints):
            rospy.loginfo("Todos los waypoints alcanzados. Deteniendo robot.")
            stop_robot()
            break

        current_waypoint = waypoints[current_waypoint_index]
        move_to_waypoint(current_waypoint)
        check_stuck()

        rospy.loginfo(
            f"x={current_x:.2f}, y={current_y:.2f}, theta={current_theta:.2f}, "
            f"pitch={current_pitch:.3f}, roll={current_roll:.3f}, "
            f"v_cmd={last_linear_speed:.2f} m/s, v_real={real_speed:.2f} m/s"
        )

        dx = current_waypoint[0] - current_x
        dy = current_waypoint[1] - current_y
        distance = sqrt(dx**2 + dy**2)

        if distance < WAYPOINT_TOLERANCE:
            rospy.loginfo(f"Waypoint {current_waypoint} alcanzado. Deteniendo 2 segundos.")
            stop_robot()
            rospy.sleep(2.0)

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
