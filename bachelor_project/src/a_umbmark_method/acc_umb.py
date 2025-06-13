#!/usr/bin/env python3
import rospy
import math
import actionlib
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

# Global state for current robot pose (updated via Odometry callback)
current_yaw = 0.0
current_x = 0.0
current_y = 0.0

# Nominal robot parameters (configure as needed)
wheelbase_nominal = 0.16   # nominal wheelbase (meters) between wheels
side_length = 2.0          # side length of the square path (meters)

cmd_pub = None   # will be initialized in main
mb_client = None # move_base client

def odom_callback(msg):
    """Odometry callback: updates current_x, current_y, current_yaw."""
    global current_x, current_y, current_yaw
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y**2 + q.z**2)
    current_yaw = math.atan2(siny_cosp, cosy_cosp)

def normalize_angle(angle):
    """Normalize an angle to [-pi, pi] range."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def go_straight(distance, speed=0.2):
    """Drive straight for the given distance (m) using open-loop timing."""
    twist = Twist()
    twist.linear.x = math.copysign(speed, distance)
    twist.angular.z = 0.0
    duration = abs(distance / speed)
    start_time = rospy.Time.now().to_sec()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown() and rospy.Time.now().to_sec() - start_time < duration:
        cmd_pub.publish(twist)
        rate.sleep()
    twist.linear.x = 0.0
    cmd_pub.publish(twist)
    rospy.sleep(0.5)

def turn(angle, angular_speed=0.2):
    """Rotate in place by the given angle (radians). CCW positive, CW negative."""
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = math.copysign(angular_speed, angle)
    duration = abs(angle / angular_speed)
    start_time = rospy.Time.now().to_sec()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown() and rospy.Time.now().to_sec() - start_time < duration:
        cmd_pub.publish(twist)
        rate.sleep()
    twist.angular.z = 0.0
    cmd_pub.publish(twist)
    rospy.sleep(0.5)

def run_square(direction, lin_speed=0.2, ang_speed=0.2):
    """Execute a square path in the specified direction ('CW' or 'CCW').
    Returns the heading error (rad) for that run."""
    init_yaw = current_yaw
    for _ in range(4):
        go_straight(side_length, speed=lin_speed)
        if direction.upper() == 'CW':
            turn(-math.pi/2, angular_speed=ang_speed)
        else:
            turn( math.pi/2, angular_speed=ang_speed)
    return normalize_angle(current_yaw - init_yaw)

def estimate_calibration(cw_error, ccw_error):
    """Compute Eb and Ed from CW/CCW final orientation errors."""
    eps_alpha = (cw_error - ccw_error) / 8.0
    eps_beta  = (cw_error + ccw_error) / 8.0
    E_b = 1.0 + (2.0 * eps_alpha) / math.pi
    if E_b != 0.0:
        delta_d = (eps_beta * wheelbase_nominal) / (2.0 * side_length * E_b)
    else:
        delta_d = 0.0
    if abs(delta_d) < 1.0:
        E_d = (1.0 - delta_d) / (1.0 + delta_d)
    else:
        E_d = float('nan')
    return E_b, E_d

def send_move_base_goal(x, y, yaw):
    """Send a 2D pose goal to move_base and wait for result."""
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "odom"    # ← changed from "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    q = quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation = Quaternion(*q)
    mb_client.send_goal(goal)
    mb_client.wait_for_result()
    return mb_client.get_state()


if __name__ == '__main__':
    rospy.init_node('diff_drive_calibration', anonymous=True)
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    mb_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base...")
    mb_client.wait_for_server()
    rospy.sleep(1.0)

    rospy.loginfo("Starting CW square path...")
    cw_error = run_square('CW')
    rospy.loginfo("Completed CW run, heading error = %.2f°", math.degrees(cw_error))

    rospy.loginfo("Returning to origin via move_base...")
    send_move_base_goal(0.0, 0.0, 0.0)
    rospy.loginfo("Arrived at origin, waiting 1s before CCW run...")
    rospy.sleep(1.0)

    rospy.loginfo("Starting CCW square path...")
    ccw_error = run_square('CCW')
    rospy.loginfo("Completed CCW run, heading error = %.2f°", math.degrees(ccw_error))

    Eb, Ed = estimate_calibration(cw_error, ccw_error)
    rospy.loginfo("Estimated E_b (wheelbase ratio) = %.5f", Eb)
    rospy.loginfo("Estimated E_d (wheel diameter ratio) = %.5f", Ed)

    # Plot final heading errors
    plt.figure()
    plt.bar(['CW run', 'CCW run'],
            [math.degrees(cw_error), math.degrees(ccw_error)],
            color=['steelblue', 'sandybrown'])
    plt.title('Final Heading Errors')
    plt.ylabel('Orientation error (degrees)')
    plt.grid(axis='y')

    # Plot calibration parameters
    plt.figure()
    plt.bar(['E_b', 'E_d'], [Eb, Ed], color=['seagreen', 'firebrick'])
    plt.title('Estimated Calibration Parameters')
    plt.ylabel('Parameter value')
    plt.ylim(0.90, 1.10)
    plt.grid(axis='y')

    plt.show()
