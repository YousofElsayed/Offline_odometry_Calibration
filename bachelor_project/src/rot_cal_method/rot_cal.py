#!/usr/bin/env python
######################################################
# 1. Library Imports
######################################################
import rospy                                     # ROS Python client library                    
import rosbag                                    # For recording/reading rosbag files (logging ROS topics)
import time                                      # Standard time library (sleep, etc.)
import yaml                                      # For saving/loading YAML files
import math                                      # Math utilities (radians, atan2, etc.)
import rospkg                                    # For locating ROS package paths
import csv                                       # For saving CSV data
import os                                        # For file and directory manipulation
import tf

from geometry_msgs.msg import Twist              # For sending velocity commands
from nav_msgs.msg import Odometry                # For odometry feedback
from sensor_msgs.msg import JointState           # For joint state feedback (wheel encoder info)
from gazebo_msgs.msg import ModelStates          # For true robot pose from Gazebo simulation
from tf2_msgs.msg import TFMessage               # For TF tree info (not strictly needed here)
from threading import Lock                       # For thread-safe rosbag writing
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

# Change the values of n, of tomasi_todt_calib_results_n_.yaml file name, of data path, of bag data path, all_endpoints_combined_n_1.csv file name. 

robot_id = 1  
N_REPEATS = 2
SIDE_LENGTH = 2 # not actually side lenght
MODE = 'opt_l'
LIN_SPEED = 0.2
ANG_SPEED = 0.25

if robot_id == 1:
    ROBOT_MODEL = 'turtlebot3_burger'
    WHEEL_BASE = 0.16
    WHEEL_RADIUS_RIGHT = 0.033
    WHEEL_RADIUS_LEFT = 0.033 
elif robot_id == 2:
    ROBOT_MODEL = 'turtlebot3_waffle'
    WHEEL_BASE = 0.287
    WHEEL_RADIUS_RIGHT = 0.033
    WHEEL_RADIUS_LEFT = 0.033 
elif robot_id == 3:
    ROBOT_MODEL = 'p3dx'
    WHEEL_BASE = 0.3
    WHEEL_RADIUS_RIGHT = 0.09
    WHEEL_RADIUS_LEFT = 0.09
elif robot_id == 4:
    ROBOT_MODEL = 'fetch'
    WHEEL_BASE = 0.374
    WHEEL_RADIUS_RIGHT = 0.098
    WHEEL_RADIUS_LEFT = 0.098
else:
    raise ValueError("Invalid robot ID. Choose 1, 2, 3, or 4.")

if MODE == 'opt_l':
    ENDING = f'sim_l_{ROBOT_MODEL}'
elif MODE == 'opt_n':
    ENDING = f'sim_n_{N_REPEATS}'


ODOM_TOPIC = "/odom"
CMD_VEL_TOPIC = "/cmd_vel"
JOINT_STATES_TOPIC = "/joint_states"
MODEL_STATES_TOPIC = "/gazebo/model_states"
TF_TOPIC = "/tf"

######################################################
# 3. Path Setup (for saving data)
######################################################
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('bachelor_project')
#data_dir = pkg_path + '/calib_data/rot_cal/turtlebot3_burger/sim_n_2'   ### CHANGE ME ### ### CHANGE ME ###
data_dir = os.path.join(pkg_path, f'calib_data/rot_cal/{ROBOT_MODEL}/{ENDING}')
os.makedirs(data_dir, exist_ok=True)
#BAG_PATH = pkg_path + '/calib_data/rot_cal/turtlebot3_burger/sim_n_2/rot_cal_n_2.bag'    ### CHANGE ME ### ### CHANGE ME ###
bag_path = os.path.join(data_dir, f'rot_cal_{ENDING}.bag')

######################################################
# 4. Rotational Calibration Class
######################################################

class Rotational_Calibration:
    def __init__(self):
        #rospy.init_node('rot_cal', anonymous=True)
        rospy.init_node('rot_cal')
        self.vel_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1) #rospy.Publisher(topic name, data class, subscriber listener)
        self.odom_sub = rospy.Subscriber(ODOM_TOPIC, Odometry, self.odom_callback) #rospy.Subscriber(topic name, data class, callback function)
        self.joint_state_sub = rospy.Subscriber(JOINT_STATES_TOPIC, JointState, self.joint_state_callback)
        self.model_states_sub = rospy.Subscriber(MODEL_STATES_TOPIC, ModelStates, self.model_states_callback)
        self.tf_sub = rospy.Subscriber(TF_TOPIC, TFMessage, self.tf_callback)
        
        self.model_name = ROBOT_MODEL
        self.reset_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        # Trajectory logs for each run (4 types: cw/ccw, 360/180)
        self.cw_360_logs_list = []          # List of trajectories for each 360deg CW run
        self.ccw_360_logs_list = []         # List of trajectories for each 360deg CCW run
        self.cw_180_logs_list = []          # List of trajectories for each 180deg CW run
        self.ccw_180_logs_list = []         # List of trajectories for each 180deg CCW run
        # Latest received ROS messages
        self.latest_odom = None
        self.latest_joint_states = None
        self.latest_model_states = None
        self.latest_tf = None

        self.sim_start_time = None
        self.sim_end_time = None
        self.total_sim_time = 0.0
        self.total_driven_distance = 0.0
        # Threading lock for rosbag writing
        self.lock = Lock()
        self.bag = rosbag.Bag(bag_path, 'w')   #w ... writing   r ... reading

    # ---- ROS Callback functions ----
    def odom_callback(self, msg):
        self.latest_odom = msg
    def joint_state_callback(self, msg):
        self.latest_joint_states = msg
    def model_states_callback(self, msg):
        self.latest_model_states = msg
    def tf_callback(self, msg):
        self.latest_tf = msg

    def reset_turtlebot_pose(self, x=0.0, y=0.0, yaw=0.0):
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


    # ---- Helper functions ----
    def stop(self):
        self.vel_pub.publish(Twist()) # Publish zero velocities (stop robot)
        rospy.sleep(0.5)


    def get_yaw_from_odom(self):
        # Extract current yaw (heading) from odometry quaternion
        if self.latest_odom is None:
            return 0.0
        q = self.latest_odom.pose.pose.orientation
        return self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def quaternion_to_yaw(self, x, y, z, w):
        # Convert quaternion (x,y,z,w) to yaw angle (in radians)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    #def get_ground_truth_pose(self):
    #    # Get true robot pose from Gazebo ModelStates message
    #    if self.latest_model_states is not None:
    #        idx = 0
    #        for i, name in enumerate(self.latest_model_states.name):
    #            if 'turtlebot3' in name:
    #                idx = i
    #                break
    #        model_pose = self.latest_model_states.pose[idx]
    #        return [model_pose.position.x, model_pose.position.y]
    #    else:
    #        return [0, 0]

    def get_ground_truth_pose(self):
        if self.latest_model_states:
            idx = self.latest_model_states.name.index(self.model_name)
            pose = self.latest_model_states.pose[idx]
            return [pose.position.x, pose.position.y]
        else:
            return [0, 0]

    def write_to_bag(self):
        # Save latest received ROS messages to rosbag (thread-safe)
        with self.lock:
            if self.latest_odom:
                self.bag.write(ODOM_TOPIC, self.latest_odom)
            if self.latest_joint_states:
                self.bag.write(JOINT_STATES_TOPIC, self.latest_joint_states)
            if self.latest_model_states:
                self.bag.write(MODEL_STATES_TOPIC, self.latest_model_states)
            if self.latest_tf:
                self.bag.write(TF_TOPIC, self.latest_tf)

    # ---- Driving and Logging ----
    def drive_rotation(self, direction, angle_deg, angular_speed=ANG_SPEED):
        # Rotates robot by given angle (CW or CCW), logging full ground-truth trajectory
        angle_rad = math.radians(angle_deg)                 # Convert target angle to radians
        twist = Twist()                                     # Create empty Twist message
        twist.angular.z = -abs(angular_speed) if direction == 'cw' else abs(angular_speed) # Set angular velocity direction
        rospy.sleep(0.5)                                    # Let sensors settle
        prev_yaw = self.get_yaw_from_odom()                 # Starting heading 
        total_angle = 0                                     # Cumulative rotation
        r = rospy.Rate(50)                                  # Loop at 50 Hz
        traj_log = []                                       # List to log pose at each timestep
        driven_dist = 0.0
        prev_pose = self.get_ground_truth_pose()

        while not rospy.is_shutdown() and abs(total_angle) < abs(angle_rad):
            #rospy.sleep(1)
            self.vel_pub.publish(twist)                     # Send rotation command
            curr_yaw = self.get_yaw_from_odom()             # Read new heading 
            d_theta = self.angle_diff(curr_yaw, prev_yaw)   # Compute how much we've rotated since last step
            total_angle = total_angle + d_theta             # Add to total
            prev_yaw = curr_yaw                             # Save for next step 
            self.write_to_bag()                      # Log all latest ROS messages    
            
            pose = self.get_ground_truth_pose()             # Get true pose (from Gazebo)
            dx = pose[0] - prev_pose[0]
            dy = pose[1] - prev_pose[1]
            d_dist = math.sqrt(dx**2 + dy**2)
            driven_dist =  driven_dist + d_dist
            prev_pose = pose
            traj_log.append(pose)                           # Save it to trajectory log
            r.sleep()                                       # Sleep for 1/50 sec 

        self.stop()                                         # Stop robot at end
        rospy.sleep(1.0)                                    # Let everything settle
        #self.total_driven_distance = self.total_driven_distance + driven_dist
        angle_rad = abs(math.radians(angle_deg))
        arc_length_per_wheel = (WHEEL_BASE / 2.0) * angle_rad   #arc_length_per_wheel=2WHEEL_BASE​⋅∣θ∣
        self.total_driven_distance = self.total_driven_distance + arc_length_per_wheel
        
        return traj_log                                     # Return full trajectory of this run

    def angle_diff(self, a, b):
        # Calculate smallest difference between two angles (handling wrap-around at 2pi)
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d

    # ---- File Saving ----
    def save_trajectory_csv(self, log, filename):
        # Save list of [x, y] to a CSV file
        with open(filename, 'w', newline='') as f: # in writing mode # same as open numpy as np
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])
            for p in log:
                writer.writerow(p)

    def save_endpoints_csv(self, endpoints, filename):
        # Save endpoints (final poses) as CSV
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])
            for p in endpoints:
                writer.writerow(p)

    def combine_csvs(self, input_dir, output_csv, pattern='endpoint'):
        import glob
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



    def perform_rotational_calibration(self):
        self.cw_360_logs_list = []
        self.ccw_360_logs_list = []
        self.cw_180_logs_list = []
        self.ccw_180_logs_list = []

        rospy.sleep(2)
        self.sim_start_time = rospy.get_time()
        
        # CW 360
        for i in range(N_REPEATS):
            self.stop()
            rospy.sleep(1.0)
            self.reset_turtlebot_pose(0.0, 0.0, 0.0)
            rospy.sleep(1.0)
            print("Robot pose after reset:", self.get_ground_truth_pose())
            rospy.sleep(1.0)
            rospy.loginfo(f"CW 360 Rotation {i+1}")
            cw_traj = self.drive_rotation('cw', 360)
            self.cw_360_logs_list.append(cw_traj)
            self.save_trajectory_csv(cw_traj, os.path.join(data_dir, f'cw_360_traj_n_{i+1}.csv'))
            print("CW endpoint", i+1, cw_traj[-1])
            self.save_endpoints_csv([cw_traj[-1]], os.path.join(data_dir, f'cw_360_endpoint_n_{i+1}.csv'))


        # CCW 360
        for i in range(N_REPEATS):
            self.stop()
            rospy.sleep(1.0)
            self.reset_turtlebot_pose(0.0, 0.0, 0.0)
            rospy.sleep(1.0)
            print("Robot pose after reset:", self.get_ground_truth_pose())
            rospy.sleep(1.0)
            rospy.loginfo(f"CCW 360 Rotation {i+1}")
            ccw_traj = self.drive_rotation('ccw', 360)
            self.ccw_360_logs_list.append(ccw_traj)
            self.save_trajectory_csv(ccw_traj, os.path.join(data_dir, f'ccw_360_traj_n_{i+1}.csv'))
            print("CCW endpoint", i+1, ccw_traj[-1])
            self.save_endpoints_csv([ccw_traj[-1]], os.path.join(data_dir, f'ccw_360_endpoint_n_{i+1}.csv'))

        # CW 180
        for i in range(N_REPEATS):
            self.stop()
            rospy.sleep(1.0)
            self.reset_turtlebot_pose(0.0, 0.0, 0.0)
            rospy.sleep(1.0)
            print("Robot pose after reset:", self.get_ground_truth_pose())
            rospy.sleep(1.0)
            rospy.loginfo(f"CW 180 Rotation {i+1}")
            cw_traj = self.drive_rotation('cw', 180)
            self.cw_180_logs_list.append(cw_traj)
            self.save_trajectory_csv(cw_traj, os.path.join(data_dir, f'cw_180_traj_n_{i+1}.csv'))
            print("CW endpoint", i+1, cw_traj[-1])
            self.save_endpoints_csv([cw_traj[-1]], os.path.join(data_dir, f'cw_180_endpoint_n_{i+1}.csv'))

        # CCW 180
        for i in range(N_REPEATS):
            self.stop()
            rospy.sleep(1.0)
            self.reset_turtlebot_pose(0.0, 0.0, 0.0)
            rospy.sleep(1.0)
            print("Robot pose after reset:", self.get_ground_truth_pose())
            rospy.sleep(1.0)
            rospy.loginfo(f"CCW 180 Rotation {i+1}")
            ccw_traj = self.drive_rotation('ccw', 180)
            self.ccw_180_logs_list.append(ccw_traj)
            self.save_trajectory_csv(ccw_traj, os.path.join(data_dir, f'ccw_180_traj_n_{i+1}.csv'))
            print("CCW endpoint", i+1, ccw_traj[-1])
            self.save_endpoints_csv([ccw_traj[-1]], os.path.join(data_dir, f'ccw_180_endpoint_n_{i+1}.csv'))

        self.reset_turtlebot_pose(0.0, 0.0, 0.0)
        self.sim_end_time = rospy.get_time()
        self.total_sim_time = self.sim_end_time - self.sim_start_time
        
        self.bag.close()
        self.save_endpoints_csv([traj[-1] for traj in self.cw_360_logs_list], os.path.join(data_dir, 'cw_360_endpoints.csv'))
        self.save_endpoints_csv([traj[-1] for traj in self.ccw_360_logs_list], os.path.join(data_dir, 'ccw_360_endpoints.csv'))
        self.save_endpoints_csv([traj[-1] for traj in self.cw_180_logs_list], os.path.join(data_dir, 'cw_180_endpoints.csv'))
        self.save_endpoints_csv([traj[-1] for traj in self.ccw_180_logs_list], os.path.join(data_dir, 'ccw_180_endpoints.csv'))
        self.combine_csvs(data_dir, os.path.join(data_dir, f'all_endpoints_combined_n_{N_REPEATS}.csv'), pattern='endpoint')    ### CHANGE ME ### ### CHANGE ME ###
        
        self.compute_tomasi_todt()
        rospy.loginfo("Calibration complete.")
        rospy.sleep(2.0)

        # No more export_to_yaml, only correct yaml write in compute_tomasi_todt

    def compute_tomasi_todt(self):
        # Extract end-points
        # Use only final poses of each 360/180 run to compute calibration
        Xcw = [traj[-1][0] for traj in self.cw_360_logs_list]
        Ycw = [traj[-1][1] for traj in self.cw_360_logs_list]
        Xccw = [traj[-1][0] for traj in self.ccw_360_logs_list]
        Yccw = [traj[-1][1] for traj in self.ccw_360_logs_list]

        print("All CCW 360 X:", Xccw)
        print("All CCW 360 Y:", Yccw)

        Xcw_mean = sum(Xcw)/len(Xcw) if Xcw else 0.0  #if the list Xcw is not empty
        Ycw_mean = sum(Ycw)/len(Ycw) if Ycw else 0.0
        Xccw_mean = sum(Xccw)/len(Xccw) if Xccw else 0.0
        Yccw_mean = sum(Yccw)/len(Yccw) if Yccw else 0.0

        # Tomasi & Todt formulas: theta for each direction
        theta_cw = math.atan2(Xcw_mean - Xccw_mean, Ycw_mean - Yccw_mean)
        theta_ccw = math.atan2(Xccw_mean - Xcw_mean, Yccw_mean - Ycw_mean)

        # Corrected Baseline per direction (CW/CCW)
        BaseRate_cw = -0.159 * theta_cw + 0.999
        BaseRate_ccw = 0.159 * theta_ccw + 0.999
        Baseline_cw = WHEEL_BASE * BaseRate_cw
        Baseline_ccw = WHEEL_BASE * BaseRate_ccw

        # Wheel diameter calibration from 180deg runs (use mean of CW, could use both)
        Y_offsets_180_cw = [traj[-1][1] for traj in self.cw_180_logs_list]
        Y_offsets_180_ccw = [traj[-1][1] for traj in self.ccw_180_logs_list]
        
        # Use mean of both directions for WheelRate
        Y_180_mean = 0.5*((sum(Y_offsets_180_cw)/len(Y_offsets_180_cw) if Y_offsets_180_cw else 0.0) + (sum(Y_offsets_180_ccw)/len(Y_offsets_180_ccw) if Y_offsets_180_ccw else 0.0))
        WheelRate = -0.01451 * Y_180_mean + 1
        R_nom = (WHEEL_RADIUS_LEFT + WHEEL_RADIUS_RIGHT) / 2.0
        Rleft_new = 2/(WheelRate+1)*R_nom
        Rright_new = 2/(1.0/WheelRate+1)*R_nom
        self.calib_results = {
            'theta_cw': float(theta_cw),
            'theta_ccw': float(theta_ccw),
            'BaseRate_cw': float(BaseRate_cw),
            'BaseRate_ccw': float(BaseRate_ccw),
            'Baseline_cw': float(Baseline_cw),
            'Baseline_ccw': float(Baseline_ccw),
            'WheelRate': float(WheelRate),
            'Rleft_new': float(Rleft_new),
            'Rright_new': float(Rright_new),
            'total_sim_time': float(self.total_sim_time),
            'total_driven_distance': float(self.total_driven_distance),
        }
        print("Tomasi & Todt Calibration Results:")
        print(yaml.dump(self.calib_results, default_flow_style=False))
        # Save results to YAML
        with open(os.path.join(data_dir, f'tomasi_todt_cal_results_{ENDING}.yaml'), 'w') as f:     ### CHANGE ME ### ### CHANGE ME ###
            yaml.dump(self.calib_results, f, default_flow_style=False)

######################################################
# 5. Script Entry Point
######################################################
if __name__ == '__main__':
    calib = Rotational_Calibration()
    rospy.sleep(2.0)
    calib.perform_rotational_calibration()
    print(f"Calibration finished. Data saved to CSV in 'calib_data/rot_cal/{ROBOT_MODEL}/{ENDING}'. Run the analysis script for plotting.")
