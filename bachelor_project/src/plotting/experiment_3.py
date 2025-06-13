#!/usr/bin/env python3
import rospy
import math
import csv
import os
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState


METHOD = 3


# Kinematic parameters (from calibration)
if METHOD == 1:
    MODE = 'bostani'
    WHEEL_RADIUS_LEFT = 0.03228762930392117
    WHEEL_RADIUS_RIGHT = 0.03371237069607884
    WHEEL_BASE = 0.15966410538407322 

elif METHOD == 2:
    MODE = 'a_umb'
    WHEEL_RADIUS_LEFT = 0.033
    WHEEL_RADIUS_RIGHT = 0.033
    WHEEL_BASE = 0.160

elif METHOD == 3:
    MODE = 'rot'
    WHEEL_RADIUS_LEFT = 0.03300
    WHEEL_RADIUS_RIGHT = 0.03299
    WHEEL_BASE = 0.1522

ANGULAR_SPEED = 0.25   # srad/s  
LIN_SPEED = 0.15       # m/s     



class experiment_paths:
    def __init__(self, wheel_radius_left, wheel_radius_right, wheel_base):
        # ROS node initialization
        rospy.init_node('experiment_path_comparison')
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.latest_odom = None
        self.reset_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.model_name = 'turtlebot3_burger'
        self.data_dir = '/home/yousof/catkin_ws/src/bachelor_project/results/final_pose_error'
        os.makedirs(self.data_dir, exist_ok=True)
        self.r_left = wheel_radius_left
        self.r_right = wheel_radius_right
        self.w_base = wheel_base

    def odom_callback(self, msg):
        self.latest_odom = msg

    def reset_pose(self, x=0.0, y=0.0, yaw=0.0):
        rospy.wait_for_service('/gazebo/set_model_state')
        state_msg = ModelState()
        state_msg.model_name = self.model_name
        state_msg.pose.position.x = x
        state_msg.pose.position.y = y
        state_msg.pose.position.z = 0
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        state_msg.pose.orientation.x = q[0]                     #_ModelState.py: MSG: geometry_msgs/Quaternion # This represents an orientation in free space in quaternion form. float64 x  float64 y  float64 z  float64 w
        state_msg.pose.orientation.y = q[1]
        state_msg.pose.orientation.z = q[2]
        state_msg.pose.orientation.w = q[3]
        state_msg.twist.linear.x = 0                             # _ModelState.py: MSG: geometry_msgs/Twist  # This expresses velocity in free space broken into its linear and angular parts. Vector3 linear  Vector3 angular 
        state_msg.twist.linear.y = 0
        state_msg.twist.linear.z = 0
        state_msg.twist.angular.x = 0
        state_msg.twist.angular.y = 0
        state_msg.twist.angular.z = 0
        state_msg.reference_frame = 'world'
        self.reset_srv(state_msg)

    def get_pose_from_odom(self):
        pos = self.latest_odom.pose.pose.position
        q = self.latest_odom.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return pos.x, pos.y, yaw  # Return [x, y, yaw]

    def stop(self):
        self.vel_pub.publish(Twist())
        rospy.sleep(0.5)

    def save_trajectory_csv(self, traj_log, filename):
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])
            for row in traj_log:
                writer.writerow(row)

                # [x, y, yaw]
# get_pose_from_odom() returns [x, y, yaw]. This ignores the first two values and only keep yaw

    def drive_to_point(self, goal_x, goal_y, v=LIN_SPEED, log_traj=None, pos_eps=0.03, heading_kp=1.5):
        while self.latest_odom is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        r = rospy.Rate(10)
        traj_log = []
        while not rospy.is_shutdown():
            curr_x, curr_y, curr_yaw = self.get_pose_from_odom()
            dx = goal_x - curr_x
            dy = goal_y - curr_y
            distance = math.sqrt(dx*dx + dy*dy)
            if distance < pos_eps:
                break

            angle_to_goal = math.atan2(dy, dx)
            heading_error = math.atan2(math.sin(angle_to_goal - curr_yaw), math.cos(angle_to_goal - curr_yaw))

            twist = Twist()
            twist.linear.x = v
            twist.angular.z = heading_kp * heading_error

            # Limit angular velocity to avoid overshoot
            max_ang = 0.5  # rad/s limit (you can adjust)
            twist.angular.z = max(min(twist.angular.z, max_ang), -max_ang)

            self.vel_pub.publish(twist)

            traj_log.append([curr_x, curr_y])
            r.sleep()
        self.stop()
        if log_traj is not None:
            log_traj.extend(traj_log)



    def rotate(self, target_yaw, w=ANGULAR_SPEED, log_traj=None, angle_eps=math.radians(1.0), angular_kp=1.5):
        while self.latest_odom is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        r = rospy.Rate(10)
        traj_log = []
        while not rospy.is_shutdown():
            _, _, curr_yaw = self.get_pose_from_odom()
            angle_error = math.atan2(math.sin(target_yaw - curr_yaw), math.cos(target_yaw - curr_yaw))
            if abs(angle_error) < angle_eps:
                break

            twist = Twist()
            twist.angular.z = angular_kp * angle_error

            # Limit angular velocity to max speed
            twist.angular.z = max(min(twist.angular.z, w), -w)

            self.vel_pub.publish(twist)
            curr_x, curr_y, _ = self.get_pose_from_odom()
            traj_log.append([curr_x, curr_y])
            r.sleep()
        self.stop()
        if log_traj is not None:
            log_traj.extend(traj_log)


    def drive_arc(self, radius, angle_rad, v=LIN_SPEED, log_traj=None, angle_eps=math.radians(2.0), distance_eps=0.02, time_margin=1.5):
        while self.latest_odom is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        start_x, start_y, start_yaw = self.get_pose_from_odom()

        arc_length = abs(radius) * abs(angle_rad)
        expected_time = (arc_length / v) * time_margin
        start_time = rospy.Time.now()
        omega = v / abs(radius) * (1 if radius > 0 else -1)
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = omega
        r = rospy.Rate(10)
        traj_log = []

        while not rospy.is_shutdown():
            self.vel_pub.publish(twist)
            curr_x, curr_y, curr_yaw = self.get_pose_from_odom()
            dx = curr_x - start_x
            dy = curr_y - start_y
            traveled = math.hypot(dx, dy)
            rotated = math.atan2(math.sin(curr_yaw - start_yaw), math.cos(curr_yaw - start_yaw))
            traj_log.append([curr_x, curr_y])
            if abs(rotated - angle_rad) < angle_eps:
                break
            if abs(traveled - arc_length) < distance_eps:
                break
            elapsed_time = (rospy.Time.now() - start_time).to_sec()
            if elapsed_time >= expected_time:
                rospy.logwarn("Stopping arc due to timeout after {:.2f}s".format(elapsed_time))
                break
            r.sleep()

        self.stop()
        if log_traj is not None:
            log_traj.extend(traj_log)

#####################################

    def run_circle(self, d=2.5, v=LIN_SPEED):
        traj_log = []
        r = d / 2.0
        angle = 2 * math.pi
        self.reset_pose(x=r, y=0.0, yaw=math.pi/2)
        rospy.sleep(1.0)
        self.drive_arc(r, angle, v, log_traj=traj_log)
        self.save_trajectory_csv(traj_log, os.path.join(self.data_dir, f'circle_traj_{MODE}.csv'))
        self.save_trajectory_csv([traj_log[-1]], os.path.join(self.data_dir, f'circle_endpoint_{MODE}.csv'))

        rospy.sleep(0.5)
        self.reset_pose()
        rospy.sleep(0.5)



    def run_triangle(self, p1, p2, v=LIN_SPEED):
        traj_log = []
        # Calculation of third point of equilateral triangle
        x1, y1 = p1
        x2, y2 = p2
        dx = x2 - x1
        dy = y2 - y1
        side_length = math.sqrt(dx*dx + dy*dy)
        mx = (x1 + x2) / 2.0
        my = (y1 + y2) / 2.0
        h = side_length * math.sqrt(3) / 2.0
        perp_dx = -dy
        perp_dy = dx
        perp_length = math.sqrt(perp_dx**2 + perp_dy**2)
        perp_dx /= perp_length
        perp_dy /= perp_length
        x3 = mx + perp_dx * h / 2.0
        y3 = my + perp_dy * h / 2.0
        triangle = [p1, p2, (x3, y3), p1]

        # Reset pose facing towards first segment
        self.reset_pose(x1, y1, math.atan2(dy, dx))
        rospy.sleep(1.0)

        for i in range(1, len(triangle)):
            goal_x, goal_y = triangle[i]
            self.drive_to_point(goal_x, goal_y, v, log_traj=traj_log)

            # Rotate to next heading if not the last point
            if i < len(triangle) - 1:
                curr_x, curr_y, curr_yaw = self.get_pose_from_odom()
                target_yaw = curr_yaw + (2 * math.pi / 3)
                # Normalize yaw
                target_yaw = math.atan2(math.sin(target_yaw), math.cos(target_yaw))
                self.rotate(target_yaw)
            rospy.sleep(0.2)

        self.save_trajectory_csv(traj_log, os.path.join(self.data_dir, f'triangle_traj_{MODE}.csv'))
        self.save_trajectory_csv([traj_log[-1]], os.path.join(self.data_dir, f'triangle_endpoint_{MODE}.csv'))
        self.reset_pose()
        rospy.sleep(0.5)


    def run_rectangle(self, rect_points, v=LIN_SPEED):
        traj_log = []
        self.reset_pose(rect_points[0][0], rect_points[0][1], 0.0)
        rospy.sleep(1.0)

        for i in range(1, len(rect_points)):
            goal_x, goal_y = rect_points[i]
            self.drive_to_point(goal_x, goal_y, v, log_traj=traj_log)

            if i < len(rect_points) - 1:
                dx = rect_points[i+1][0] - goal_x
                dy = rect_points[i+1][1] - goal_y
                angle_to_next = math.atan2(dy, dx)
                self.rotate(angle_to_next)
            rospy.sleep(0.2)

        self.save_trajectory_csv(traj_log, os.path.join(self.data_dir, f'rectangle_traj_{MODE}.csv'))
        self.save_trajectory_csv([traj_log[-1]], os.path.join(self.data_dir, f'rectangle_endpoint_{MODE}.csv'))

        self.reset_pose()
        rospy.sleep(0.5)



    def run_figure_eight(self, d=2.5, v=LIN_SPEED):
        traj_log = []
        r = d / 2.0
        self.reset_pose(x=r, y=0.0, yaw=math.pi/2)
        rospy.sleep(1.0)

        self.drive_arc(r, 2 * math.pi, v, log_traj=traj_log)   # CCW
        self.drive_arc(-r, 2 * math.pi, v, log_traj=traj_log)  # CW


        self.save_trajectory_csv(traj_log, os.path.join(self.data_dir, f'figure_eight_traj_{MODE}.csv'))
        self.save_trajectory_csv([traj_log[-1]], os.path.join(self.data_dir, f'figure_eight_endpoint_{MODE}.csv'))

        rospy.sleep(0.5)
        self.reset_pose()
        rospy.sleep(0.5)


if __name__ == '__main__':
    runner = experiment_paths(WHEEL_RADIUS_LEFT, WHEEL_RADIUS_RIGHT, WHEEL_BASE)
    rospy.sleep(1.0)
    #runner.run_circle(d=5.0)
    #print("run_circle finished")
    #runner.run_triangle((0,0), (5,0))
    #print("run_triangle finished")
    #runner.run_rectangle([(0,0), (10,0), (10,5), (0,5), (0,0)])
    #print("run_rectangle finished")
    runner.run_figure_eight(d=5)
    print("run_figure_eight finished")
    print("All paths executed and saved.")


    #runner.run_triangle(side=5.0)
    #runner.run_rectangle(l1=7.0, l2=5.0)