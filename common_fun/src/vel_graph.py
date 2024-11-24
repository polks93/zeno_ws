#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
import time
from marta_msgs.msg import NavStatus
from joystick_command.msg import Rel_error_joystick
from nav_msgs.msg import Odometry
import numpy as np

class VelGraph:
    def __init__(self):
        rospy.init_node('vel_graph_node')
        self.rate = rospy.Rate(30)  # 1 Hz

        self.start_time = None
        self.recording = False
        self.record_time = 30 # seconds
        self.waiting_time = 0.5 # seconds
        self.time_stamps = []
        self.velocities = []
        self.reference = []
        
        rospy.Subscriber('odom', Odometry, self.nav_status_callback)
        rospy.Subscriber('relative_error', Rel_error_joystick, self.rel_error_joystick_callback)

    def rel_error_joystick_callback(self, msg):
        rospy.sleep(self.waiting_time)
        self.recording = True
        self.ref = msg.error_yaw
        rospy.loginfo("Recording started")

    def nav_status_callback(self, msg):
        if self.recording:
            if self.start_time is None:
                self.start_time = time.time()
            curr_time = time.time()
            elapsed_time = curr_time - self.start_time
            if elapsed_time < self.record_time:
                data = msg.twist.twist.angular.z
                self.time_stamps.append(elapsed_time)
                self.velocities.append(data)
                self.reference.append(self.ref)
            else:
                if self.recording:
                    self.recording = False
                    rospy.loginfo("Recording finished")
                    vel_vector = np.array(self.velocities)
                    time_vector = np.array(self.time_stamps)
                    ref_vector = np.array(self.reference)
                    np.savez("omega90", vel=vel_vector, time=time_vector, ref=ref_vector)
                    self.plot_graph()
    
    def plot_graph(self):
        plt.figure(figsize=(10, 6))
        plt.plot(self.time_stamps, self.velocities, label="Velocità", color='b', linestyle='-')
        # plt.plot(self.time_stamps, self.reference, label="Riferimento", color='r', linestyle='--')
        plt.title("Andamento della velocità angolare nel tempo")
        plt.xlabel("Tempo (s)")
        plt.ylabel("Velocità angoalre (rad/s)")
        plt.grid(True)
        plt.legend()
        plt.savefig("omega90.png")  # Salva il grafico come immagine
        plt.show()  # Mostra il grafico a schermo
        rospy.signal_shutdown("Grafico creato.")  # Ferma il nodo ROS

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        vel_graph = VelGraph()
        vel_graph.run()
    except rospy.ROSInterruptException:
        pass

