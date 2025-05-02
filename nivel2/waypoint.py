#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt
from tf.transformations import euler_from_quaternion

# Inicialización de variables
waypoints = [(1, 0), (3, 0), (3.5, -1), (7, -1)]
current_waypoint_index = 0
current_x, current_y, current_theta = 0.0, 0.0, 0.0

# Publicador
cmd_vel_pub = None

# Callback de Odometry
def odom_callback(msg):
    global current_x, current_y, current_theta
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    _, _, current_theta = euler_from_quaternion([
        orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
    ])

# Movimiento hacia un waypoint
def move_to_waypoint(waypoint):
    global current_x, current_y, current_theta

    dx = waypoint[0] - current_x
    dy = waypoint[1] - current_y
    distance = sqrt(dx**2 + dy**2)
    angle_to_goal = atan2(dy, dx)
    angle_diff = angle_to_goal - current_theta

    while angle_diff > 3.14159:
        angle_diff -= 2 * 3.14159
    while angle_diff < -3.14159:
        angle_diff += 2 * 3.14159

    linear_speed = min(0.4 * distance, 0.35)
    angular_speed = min(0.4 * angle_diff, 0.4)

    cmd_vel = Twist()
    cmd_vel.linear.x = linear_speed
    cmd_vel.angular.z = angular_speed
    cmd_vel_pub.publish(cmd_vel)

# Navegación principal
def navigate():
    global current_waypoint_index

    rospy.Subscriber('/odom', Odometry, odom_callback)
    rate = rospy.Rate(10)
    prev_x, prev_y = 0.0, 0.0
    stuck_timer = rospy.Time.now()
    stuck_threshold = 0.05

    while not rospy.is_shutdown():
        current_waypoint = waypoints[current_waypoint_index]
        move_to_waypoint(current_waypoint)

        dx = current_waypoint[0] - current_x
        dy = current_waypoint[1] - current_y
        distance = sqrt(dx**2 + dy**2)

        # Detección de atasco
        if (rospy.Time.now() - stuck_timer).to_sec() > 1.0:
            dx_stuck = current_x - prev_x
            dy_stuck = current_y - prev_y
            if sqrt(dx_stuck**2 + dy_stuck**2) < stuck_threshold:
                rospy.logwarn("Posible atasco. Impulso adicional.")
                cmd_vel = Twist()
                cmd_vel.linear.x = 0.4
                cmd_vel_pub.publish(cmd_vel)
            prev_x, prev_y = current_x, current_y
            stuck_timer = rospy.Time.now()

        if distance < 0.15:
            rospy.loginfo(f"Waypoint {current_waypoint} alcanzado")
            current_waypoint_index += 1
            if current_waypoint_index >= len(waypoints):
                rospy.loginfo("Todos los waypoints alcanzados. Deteniendo robot.")
                break

        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('waypoint_navigation', anonymous=True)
        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        navigate()
    except rospy.ROSInterruptException:
        pass

