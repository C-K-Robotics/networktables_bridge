#!/usr/bin/env python3
#
# A client that publishes some synchronized values periodically

# import argparse
import os
from os.path import basename
import logging
import time

import ntcore
import numpy as np

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from .nt_utils import findROSClass, msg2json, json2msg, nt_type_dict, nt_create_topic

class NTClientPub(Node):

    def __init__(self):
        super().__init__('NT_Client_Publish_Node', allow_undeclared_parameters=True)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('NT_server_ip', "10.80.20.2"),
                ('sampling_time', 0.1),
                ('sub_rostopic_names', [""]),
                ('msg_types', [""]),
                ('pub_NT_names', [""])]
        )

        # self.glb_sp_wpnts = WpntArray()
        # self.glb_sp_wpnts_sub_ = self.create_subscription(WpntArray, '/global_waypoints/shortest_path', self.glb_sp_wpnts_cb, 10)
        # self.glb_sp_wpnts_sub_

        # self.goal_update_pub_ = self.create_publisher(PoseStamped, self.get_parameter('goal_topic').value, 10)

        logging.basicConfig(level=logging.DEBUG)

        # get NT Server IP address
        ip = self.get_parameter('NT_server_ip').value

        # Initialize NT4 client
        self.inst = ntcore.NetworkTableInstance.getDefault()

        identity = f"{basename(__file__)}-{os.getpid()}"
        self.inst.startClient4(identity)

        self.inst.setServer(ip)

        self.sub_rostopic_names = self.get_parameter('sub_rostopic_names').value
        self.msg_types = self.get_parameter('msg_types').value
        self.pub_NT_names = self.get_parameter('pub_NT_names').value

        self.coerceSizeCheck()
        self.get_logger().info('NT Publishers Enabled!')

        self.create_subs()

        # Start a timer
        self.Ts = self.get_parameter('sampling_time').value
        self.timer = self.create_timer(self.Ts, self.periodic)
    
    def periodic(self):
        # TODO: periodic logic
        for msg in self.msgs:
            index = self.msgs.index(msg)
            nt_pub = self.nt_pubs[index]

            # serialize msg
            json_msg, msg_type = msg2json(msg)
            # self.get_logger().info(f'Serialize msg #{index}!')

            # publish to NetworkTable
            nt_pub.set(json_msg)
            # self.get_logger().info(f'Publish msg #{index} to NetworkTable!')

    def create_subs(self):
        self.msgs = []
        self.subs = []
        self.nt_types = []
        self.nt_pubs = []
        self.functions = []
        for rostopic_name in self.sub_rostopic_names:
            index = self.sub_rostopic_names.index(rostopic_name)
            msg_type = self.msg_types[index]
            msg_type = msg_type.replace("/", ".")
            nt_name = self.pub_NT_names[index]

            nt_type = "String"
            self.nt_types.append(nt_type)

            self.msg_types[index] = msg_type
            self.functions.append(self.create_a_function(index))
            self.get_logger().info(f'Function #{index} Created!')
            var_class = findROSClass(msg_type)
            self.msgs.append(var_class())
            self.subs.append(self.create_subscription(var_class, rostopic_name, self.functions[index], 10))
            
            # create nt publishers
            self.nt_pubs.append(nt_create_topic(self.inst, nt_type, nt_name).publish())

    def create_a_function(self, index):
        def msg_cb(msg):
            self.msgs[index] = msg
            # self.get_logger().info(f'msg #{index}: {self.msgs[index]}')
        return msg_cb

    def coerceSizeCheck(self):
        if len(self.sub_rostopic_names) == len(self.msg_types):
            if len(self.msg_types) == len(self.pub_NT_names):
                self.get_logger().info('Number of topics Checked!')
            else:
                raise Exception("Topic info has unmatched size!!! Please check you yaml file.")
        else:
            raise Exception("Topic info has unmatched size!!! Please check you yaml file.")
        
        
def main(args=None):
    rclpy.init(args=args)

    node = NTClientPub()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

    # # publish two values
    # table = inst.getTable("data")
    # pub1 = table.getDoubleTopic("1").publish()
    # pub2 = table.getDoubleTopic("2").publish()

    # i = 3

    # while True:
    #     # These values are being published fast than the server is polling
    #     pub1.set(i)
    #     pub2.set(i + 100)

    #     time.sleep(0.5)
    #     i += 1