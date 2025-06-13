#!/usr/bin/env python3
import math
import time
import rospy
import rospkg
import yaml
import csv
import os
import tf
import rosbag
import threading
import glob
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

# ======================== Robots ========================
robot_id = 1 
N_REPEATS = 1
SIDE_LENGTH = 1  # Distance to drive forward (meters)
MODE = 'opt_n'
LIN_SPEED = 0.2
ANG_SPEED = 0.4

if robot_id == 1:
    ROBOT_MODEL = 'turtlebot3_burger'
    WHEEL_BASE = 0.16
    WHEEL_RADIUS = 0.033
elif robot_id == 2:
    ROBOT_MODEL = 'turtlebot3_waffle'
    WHEEL_BASE = 0.287
    WHEEL_RADIUS = 0.033
elif robot_id == 3:
    ROBOT_MODEL = 'p3dx'
    WHEEL_BASE = 0.3
    WHEEL_RADIUS = 0.09
elif robot_id == 4:
    ROBOT_MODEL = 'fetch'
    WHEEL_BASE = 0.374
    WHEEL_RADIUS = 0.098
else:
    raise ValueError("Invalid robot ID. Choose 1, 2, 3, or 4.")

if MODE == 'opt_l':
    ENDING = f'sim_l_{SIDE_LENGTH}'
elif MODE == 'opt_n':
    ENDING = f'sim_n_{N_REPEATS}'

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('bachelor_project')
data_dir = os.path.join(pkg_path, f'calib_data/trian_cal_method/{ROBOT_MODEL}/{ENDING}')
os.makedirs(data_dir, exist_ok=True)
bag_path = os.path.join(data_dir, f'triangle_cal_data_{ENDING}.bag')

ODOM_TOPIC = "/odom"
CMD_VEL_TOPIC = "/cmd_vel"
JOINT_STATES_TOPIC = "/joint_states"
MODEL_STATES_TOPIC = "/gazebo/model_states"
TF_TOPIC = "/tf"

class TriangleCalibration:
    def __init__(self):
        rospy.init_node("triangle_calibration", anonymous=True)
        self.vel_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber(ODOM_TOPIC, Odometry, self.odom_callback)
        self.joint_state_sub = rospy.Subscriber(JOINT_STATES_TOPIC, JointState, self.joint_state_callback)
        self.model_states_sub = rospy.Subscriber(MODEL_STATES_TOPIC, ModelStates, self.model_states_callback)
        self.tf_sub = rospy.Subscriber(TF_TOPIC, TFMessage, self.tf_callback)
        self.latest_odom = None
        self.latest_joint_states = None
        self.latest_model_states = None
        self.latest_tf = None
        self.reset_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.model_name = ROBOT_MODEL
        self.lock = threading.Lock()
        self.bag = rosbag.Bag(bag_path, 'w')
        # Data collection
        self.exp_results = {'cw': [], 'ccw': []}
        self.traj_logs = {'cw': [], 'ccw': []}
        self.endpoints = {'cw': [], 'ccw': []}
        self.calib_results = {}

        self.sim_start_time = None
        self.sim_end_time = None
        self.total_sim_time = 0.0
        self.total_driven_distance = 0.0


    # --- ROS callbacks ---
    def odom_callback(self, msg):
        self.latest_odom = msg

    def joint_state_callback(self, msg):
        self.latest_joint_states = msg

    def model_states_callback(self, msg):
        self.latest_model_states = msg

    def tf_callback(self, msg):
        self.latest_tf = msg

    def write_to_bag(self):
        with self.lock:
            if self.latest_odom:
                self.bag.write(ODOM_TOPIC, self.latest_odom)
            if self.latest_joint_states:
                self.bag.write(JOINT_STATES_TOPIC, self.latest_joint_states)
            if self.latest_model_states:
                self.bag.write(MODEL_STATES_TOPIC, self.latest_model_states)
            if self.latest_tf:
                self.bag.write(TF_TOPIC, self.latest_tf)

    def get_pose_from_odom(self):
        while self.latest_odom is None and not rospy.is_shutdown():
            rospy.sleep(0.5)
        pos = self.latest_odom.pose.pose.position
        q = self.latest_odom.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return pos.x, pos.y, yaw

    def get_pose_from_ground_truth(self):
        while (self.latest_model_states is None or self.model_name not in self.latest_model_states.name) and not rospy.is_shutdown():
            rospy.sleep(0.5)
        idx = self.latest_model_states.name.index(self.model_name)
        pos = self.latest_model_states.pose[idx].position
        q = self.latest_model_states.pose[idx].orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return pos.x, pos.y, yaw


    def reset_pose(self, x=0.0, y=0.0, yaw=0.0):
        rospy.wait_for_service('/gazebo/set_model_state')
        state_msg = ModelState()
        state_msg.model_name = self.model_name
        state_msg.pose.position.x = x
        state_msg.pose.position.y = y
        state_msg.pose.position.z = 0
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        state_msg.pose.orientation.x = q[0]
        state_msg.pose.orientation.y = q[1]
        state_msg.pose.orientation.z = q[2]
        state_msg.pose.orientation.w = q[3]
        state_msg.twist.linear.x = 0
        state_msg.twist.linear.y = 0
        state_msg.twist.linear.z = 0
        state_msg.twist.angular.x = 0
        state_msg.twist.angular.y = 0
        state_msg.twist.angular.z = 0
        state_msg.reference_frame = 'world'
        self.reset_srv(state_msg)

    def stop(self):
        self.vel_pub.publish(Twist())
        rospy.sleep(0.5)

    def move_straight(self, distance, speed=LIN_SPEED, traj=None):
        x0, y0, yaw0 = self.get_pose_from_odom()
        cmd = Twist()
        rate = rospy.Rate(10)
        traveled = 0
        last_x, last_y = x0, y0

        while abs(traveled) < abs(distance) and not rospy.is_shutdown():
            _, _, yaw = self.get_pose_from_odom()
            yaw_error = math.atan2(math.sin(yaw0 - yaw), math.cos(yaw0 - yaw))  # [-pi, pi]
            cmd.linear.x = speed if distance >= 0 else -speed
            cmd.angular.z = 1.5 * yaw_error  # simple P controller on yaw

            self.vel_pub.publish(cmd)
            self.write_to_bag()
            rate.sleep()

            x, y, _ = self.get_pose_from_odom()
            step_traveled = math.sqrt((x - last_x) ** 2 + (y - last_y) ** 2)
            traveled = math.sqrt((x - x0)**2 + (y - y0)**2)
            if traj is not None:
                traj.append([x, y])
                
            self.total_driven_distance = self.total_driven_distance + step_traveled
            last_x, last_y = x, y  # update for next step
        self.stop()

    
        
    def rotate_in_place(self, angle_deg, speed=ANG_SPEED, traj=None):
        angle_rad = math.radians(angle_deg)
        twist = Twist()
        twist.angular.z = -abs(speed) if angle_rad < 0 else abs(speed)
        rospy.sleep(0.5)

        _, _, prev_yaw = self.get_pose_from_ground_truth()
        total_angle = 0.0
        r = rospy.Rate(50)

        if traj is None:
            traj = []

        prev_x, prev_y, _ = self.get_pose_from_ground_truth()
        distance_rot = 0.0

        while not rospy.is_shutdown() and abs(total_angle) < abs(angle_rad):
            self.vel_pub.publish(twist)
            _, _, curr_yaw = self.get_pose_from_ground_truth()
            d_theta = self.angle_diff(curr_yaw, prev_yaw)
            total_angle = total_angle + d_theta
            prev_yaw = curr_yaw

            x, y, _ = self.get_pose_from_ground_truth()
            traj.append([x, y])
            self.write_to_bag()
            r.sleep()

        arc_length_per_wheel = (WHEEL_BASE / 2.0) * abs(total_angle)   
        self.total_driven_distance = self.total_driven_distance + arc_length_per_wheel
        self.stop()
        rospy.sleep(1.0)


    def angle_diff(self, yaw, yaw0):
        return math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0))

    def triangle_sides(self, A, B, C):
        def dist(p1, p2):
            return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
        a = dist(B, C)  # |BC|
        b = dist(A, C)  # |AC|
        c = dist(A, B)  # |AB|
        return a, b, c

    def triangle_angle(self, a, b, c):
        if a * c == 0:
            return 0.0
        angle = math.acos(max(min((a**2 + c**2 - b**2) / (2*a*c), 1.0), -1.0))
        return angle

    def save_trajectory_csv(self, log, filename):
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])
            for p in log:
                writer.writerow(p)

    def save_endpoints_csv(self, endpoint, filename):
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])
            writer.writerow(endpoint)

    def combine_csvs(self, input_dir, output_csv, pattern='endpoint'):
        combined = []
        for f in sorted(glob.glob(os.path.join(input_dir, f'*{pattern}*.csv'))):
            with open(f, 'r') as infile:
                reader = csv.reader(infile)
                header = next(reader)
                for row in reader:
                    combined.append(row)
        with open(output_csv, 'w', newline='') as outfile:
            writer = csv.writer(outfile)
            writer.writerow(['x', 'y'])
            for row in combined:
                writer.writerow(row)

    def save_results_yaml(self):
        with open(os.path.join(data_dir, f'triangle_calib_results_{ENDING}.yaml'), 'w') as f:
            yaml.dump(self.calib_results, f, default_flow_style=False)

    def run_experiment(self, direction='cw', rep=1):
        traj = []
        self.reset_pose()
        time.sleep(1)
        A = self.get_pose_from_ground_truth()
        traj.append([A[0], A[1]])
        self.write_to_bag()

        self.move_straight(SIDE_LENGTH, traj=traj)
        B = self.get_pose_from_ground_truth()
        traj.append([B[0], B[1]])
        self.write_to_bag()

        rot_angle = -180 if direction == 'cw' else 180
        self.rotate_in_place(rot_angle, traj=traj)
        C0 = self.get_pose_from_ground_truth()
        traj.append([C0[0], C0[1]])
        self.write_to_bag()

        self.move_straight(SIDE_LENGTH, traj=traj)
        C = self.get_pose_from_ground_truth()
        traj.append([C[0], C[1]])
        self.write_to_bag()
        self.reset_pose()
        self.stop()

        self.save_trajectory_csv(traj, os.path.join(data_dir, f'{direction}_traj_n_{rep}.csv'))
        self.save_endpoints_csv([C[0], C[1]], os.path.join(data_dir, f'{direction}_endpoint_n_{rep}.csv'))
        return (A, B, C), traj, [C[0], C[1]]

    def compute_angles_from_trials(self, trials):
        angles = []
        for A, B, C in trials:
            a, b, c = self.triangle_sides(A, B, C)
            angle = self.triangle_angle(a, b, c)
            angles.append(angle)
        return angles

    def compute_calibration(self):
        cw_angles = self.compute_angles_from_trials(self.exp_results['cw'])
        ccw_angles = self.compute_angles_from_trials(self.exp_results['ccw'])
        if not cw_angles or not ccw_angles:
            print("Not enough data for calibration.")
            return

        theta1 = sum(cw_angles) / len(cw_angles)
        theta2 = sum(ccw_angles) / len(ccw_angles)
        theta_plus = (theta1 + theta2) / 2
        theta_minus = (theta1 - theta2) / 2

        M = SIDE_LENGTH
        if abs(theta_plus) > 1e-6:
            R = M / theta_plus
            Es = M / (R * theta_plus)
        else:
            R = float('inf')
            Es = 1.0

        Eb = math.pi / (math.pi - theta_minus) if abs(math.pi - theta_minus) > 1e-6 else 1.0

        bnom = WHEEL_BASE
        if abs(theta_plus) > 1e-6:
            N = 2 * R / bnom
            Ed = (bnom * N + 2 * M) / (bnom * N - 2 * M) if (bnom * N - 2 * M) != 0 else 1.0
        else:
            Ed = 1.0

        effective_wheelbase = Eb * WHEEL_BASE
        left_wheel_radius = (2 * WHEEL_RADIUS) / (Ed + 1)
        right_wheel_radius = (2 * WHEEL_RADIUS) * Ed / (Ed + 1)

        self.calib_results = {
            'Es': float(Es),
            'Eb': float(Eb),
            'Ed': float(Ed),
            'effective_wheelbase': float(effective_wheelbase),
            'left_wheel_radius': float(left_wheel_radius),
            'right_wheel_radius': float(right_wheel_radius),
            'theta1': float(theta1),
            'theta2': float(theta2),
            'theta_plus': float(theta_plus),
            'theta_minus': float(theta_minus),
            'total_sim_time': float(self.total_sim_time),
            'total_driven_distance': float(self.total_driven_distance)
        }

        self.save_results_yaml()

    def perform_triangle_calibration(self):
        rospy.sleep(1.0)
        self.sim_start_time = rospy.get_time()

        for i in range(N_REPEATS):
            rospy.loginfo(f"Starting CW experiment {i+1}")
            ABC_cw, traj_cw, endpoint_cw = self.run_experiment(direction='cw', rep=i+1)
            self.exp_results['cw'].append(ABC_cw)
            self.traj_logs['cw'].append(traj_cw)
            self.endpoints['cw'].append(endpoint_cw)
            rospy.sleep(1.0)

            rospy.loginfo(f"Starting CCW experiment {i+1}")
            ABC_ccw, traj_ccw, endpoint_ccw = self.run_experiment(direction='ccw', rep=i+1)
            self.exp_results['ccw'].append(ABC_ccw)
            self.traj_logs['ccw'].append(traj_ccw)
            self.endpoints['ccw'].append(endpoint_ccw)
            rospy.sleep(1.0)
        self.combine_csvs(data_dir, os.path.join(data_dir, 'all_endpoints_combined.csv'), pattern='endpoint')
        self.sim_end_time = rospy.get_time()
        self.total_sim_time = self.sim_end_time - self.sim_start_time
        self.compute_calibration()
        self.bag.close()
        print("Triangle calibration finished.")

if __name__ == "__main__":
    calib = TriangleCalibration()
    calib.perform_triangle_calibration()
    print("Triangle calibration finished. Data saved to CSV and YAML.")
    print(f"Data saved in '{data_dir}'. Run your plotting/analysis scripts if desired.")
    print(f"Triangle Calibration finished. Data saved to CSV in 'calib_data/trian_cal_method/{ROBOT_MODEL}/{ENDING}'. Run the analysis script for plotting.")
