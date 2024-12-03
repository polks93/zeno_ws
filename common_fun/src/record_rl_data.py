#!/usr/bin/env python

import rospy
import time

from joystick_command.msg import Rel_error_joystick
from nav_msgs.msg import Odometry
import numpy as np
from models.msg import Coverage
from std_msgs.msg import String
import tf

class RecordData:
    def __init__(self):
        rospy.init_node('record_data_node')
        self.rate = rospy.Rate(30)  # 1 Hz

        self.start_time = None
        self.recording = False

        self.coverage_timestamps = []
        self.coverage_values = []

        self.odom_timestamps = []
        self.x_values = []
        self.y_values = []
        self.yaw_values = []

        self.vx_values = []
        self.vy_values = []
        self.omega_values = []

        self.relative_error_timestamps = []
        self.v_surge_des_values = []
        self.v_sway_des_values = []
        self.yaw_rel_values = []

        self.abort_timestamps = []
        self.abort_values = []

        rospy.Subscriber('/start_scan', String, self.start_callback)
        rospy.Subscriber('/coverage', Coverage, self.coverage_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/nbv/relative_error', Rel_error_joystick, self.rel_error_joystick_callback)
        rospy.Subscriber('/save_data', String, self.save_data)
        rospy.Subscriber('/abort', String, self.abort_callback)

        # Import limiti workspace
        workspace_params = ["/workspace/x_min", "/workspace/y_min", "/workspace/x_max", "/workspace/y_max"]

        while not all(rospy.has_param(param) for param in workspace_params) and not rospy.is_shutdown():
            self.rate.sleep()

        xmin = rospy.get_param("/workspace/x_min", 0.0)
        ymin = rospy.get_param("/workspace/y_min", 0.0)
        xmax = rospy.get_param("/workspace/x_max", 10.0)
        ymax = rospy.get_param("/workspace/y_max", 10.0)
        self.workspace = (xmin, ymin, xmax, ymax)
        self.ship_scale_factor      = rospy.get_param('/ship/scale', 0.9)
        self.ship_center            = ((xmin + xmax) / 2, (ymin + ymax) / 2)

    def find_elapse_time(self, msg):
        msg_time = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9
        return msg_time - self.start_time

    def start_callback(self, msg):
        self.recording = True
        self.start_time = time.time()
        rospy.loginfo("Recording started")

    def abort_callback(self, msg):
        curr_time = time.time() - self.start_time
        self.abort_timestamps.append(curr_time)
        self.abort_values.append(msg.data)


    def coverage_callback(self, msg):
        if self.recording:
            elapsed_time = self.find_elapse_time(msg)
            self.coverage_timestamps.append(elapsed_time)
            self.coverage_values.append(msg.data)

    def odom_callback(self, msg):
        if self.recording:
            elapsed_time = self.find_elapse_time(msg)
            self.odom_timestamps.append(elapsed_time)
            self.x_values.append(msg.pose.pose.position.x)
            self.y_values.append(msg.pose.pose.position.y)
            q = msg.pose.pose.orientation
            _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.yaw_values.append(yaw)
            self.vx_values.append(msg.twist.twist.linear.x)
            self.vy_values.append(msg.twist.twist.linear.y)
            self.omega_values.append(msg.twist.twist.angular.z)


    def rel_error_joystick_callback(self, msg):
        if self.recording:
            elapsed_time = self.find_elapse_time(msg)
            self.relative_error_timestamps.append(elapsed_time)
            self.v_surge_des_values.append(msg.error_surge_speed)
            self.v_sway_des_values.append(msg.error_sway_speed)
            self.yaw_rel_values.append(msg.error_yaw)



    def save_data(self, msg):
        workspace = np.array(self.workspace)
        ship_scale_factor = np.array(self.ship_scale_factor)
        ship_center = np.array(self.ship_center)

        abort_timestamps = np.array(self.abort_timestamps)
        abort_values = np.array(self.abort_values)

        coverage_timestamps = np.array(self.coverage_timestamps)
        coverage= np.array(self.coverage_values)

        odom_timestamps = np.array(self.odom_timestamps)
        x = np.array(self.x_values)
        y = np.array(self.y_values)
        yaw = np.array(self.yaw_values)
        vx = np.array(self.vx_values)
        vy = np.array(self.vy_values)
        omega = np.array(self.omega_values)


        relative_error_timestamps = np.array(self.relative_error_timestamps)
        v_surge_des = np.array(self.v_surge_des_values)
        v_sway_des = np.array(self.v_sway_des_values)
        yaw_rel = np.array(self.yaw_rel_values)
        rospy.loginfo("Saving data")
        
        np.savez("RL_data/sim_data", 
                 abort_timestamps=abort_timestamps,
                 abort_values=abort_values,
                 coverage_timestamps=coverage_timestamps, 
                 coverage=coverage,
                 odom_timestamps=odom_timestamps, 
                 x=x,   
                 y=y, 
                 yaw=yaw, 
                 vx=vx, 
                 vy=vy,
                 omega=omega,
                 relative_error_timestamps=relative_error_timestamps, 
                 v_surge_des=v_surge_des, 
                 v_sway_des=v_sway_des,
                 yaw_rel=yaw_rel,
                 workspace=workspace,
                 ship_scale_factor=ship_scale_factor,
                 ship_center=ship_center)
        rospy.loginfo("Data saved")

    def run(self):
        rospy.spin()
if __name__ == '__main__':
    try:
        record = RecordData()
        record.run()
    except rospy.ROSInterruptException:
        pass

