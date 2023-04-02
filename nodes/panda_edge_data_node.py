#!/usr/bin/env python

import rospy
import os
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from kdmp_ros.msg import PandaControlCmd

PANDA_CONTROL_DURATION = 0.02

class PandaEdgeDataNode:
    def __init__(self, data_path, data_key="vel_mag"):
        self.data_path = data_path
        self.pub = rospy.Publisher('panda_data', PandaControlCmd, queue_size=10)
        self.data = {}
        self.load_data()

    def get_random_cmd_sample(self):
        index = np.random.randint(0, len(self.data["q"]))
        cmd = PandaControlCmd()
        cmd_data = self.data["vel_mag"][index] * self.data["vel_dir"][index]
        cmd.cmd.linear.x = cmd_data[0]
        cmd.cmd.linear.y = cmd_data[1]
        cmd.cmd.linear.z = cmd_data[2]
        cmd.cmd.angular.x = cmd_data[3]
        cmd.cmd.angular.y = cmd_data[4]
        cmd.cmd.angular.z = cmd_data[5]

        cmd.durration = self.data["num_steps"][index] * PANDA_CONTROL_DURATION
        cmd.q_start = self.data["q"][index][0]
        cmd.qd_start = self.data["dq"][index][0]
        cmd.q_end = self.data["q"][index][1]
        cmd.qd_end = self.data["dq"][index][1]
        cmd.tau = self.data["tau"][index][0]
        return cmd

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            data = self.get_random_cmd_sample()
            # print("\n\n\n\n")
            print(data)
            self.pub.publish(data)
            rate.sleep()

    def load_data(self):
        for file in os.listdir(self.data_path):
            if (file.endswith(".npy")):
                data = np.load(self.data_path + "/" + file)
                self.data[file[:-4]] = data
        if len(self.data) == 0:
            raise "No data in data directory"
        print(self.data)

if __name__ == "__main__":
    data_path = "/home/liam/data/"
    data_run = "eb_100000_20230220-203205"
    rospy.init_node("panda_edge_data_node")
    pen = PandaEdgeDataNode(data_path + data_run)
    pen.run()
    rospy.spin()
