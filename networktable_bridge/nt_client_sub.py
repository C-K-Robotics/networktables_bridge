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

from .nt_utils import findROSClass, msg2json, json2msg, nt_type_dict, ros_type_dict, nt_create_topic, nt_default_value, nt2msg

def find_all(a_str:str, sub:str):
    """Helper function to find all occurrences of a substring in a string."""
    start = 0
    while True:
        if a_str == '': return
        start = a_str.find(sub, start)
        if start == -1: return
        yield start
        start += len(sub) # use start += 1 to find overlapping matches

def is_invalid_char(char):
    """Define what constitutes an invalid character for a ROS topic name."""
    return char in "!@#$%^&*()~{,}-=+<>:"

class NTClientSub(Node):

    def __init__(self):
        super().__init__('NT_Client_Subscribe_Node', allow_undeclared_parameters=True)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('NT_server_ip', "10.80.20.2"),
                ('sampling_time', 0.1),
                ('sub_NT_names', [""]),
                ('msg_types', [""]),
                ('pub_rostopic_names', [""]),
                ('automated', True),
            ]
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

        self.sub_NT_names = []
        self.msg_types = []
        self.pub_rostopic_names = []
        self.pubs = []
        self.nt_types = []
        self.nt_subs = []
        self._on_subs = []
        self.valueListenersHandle = []
        # self.msgs = []

        self.user_defined_sub_NT_names = self.get_parameter('sub_NT_names').value
        self.sub_NT_names.extend(self.user_defined_sub_NT_names)
        self.user_defined_msg_types = self.get_parameter('msg_types').value
        self.msg_types.extend(self.user_defined_msg_types)
        self.user_defined_pub_rostopic_names = self.get_parameter('pub_rostopic_names').value
        self.pub_rostopic_names.extend(self.user_defined_pub_rostopic_names)

        self.removeEmptyEntries()
        self.coerceSizeCheck()
        self.get_logger().info('NT Subscribers Enabled!')

        if self.get_parameter('automated').value:
            self.topicListenerHandle = self.inst.addListener(
                ["/"], ntcore.EventFlags.kTopic, self._on_pub
            )
        else:
            self.create_pubs()

        self.connListenerHandle = self.inst.addConnectionListener(True, self._connect_cb)

        # Start a timer
        # self.Ts = self.get_parameter('sampling_time').value
        # self.timer = self.create_timer(self.Ts, self.periodic)
    
    def make_func_on_sub(self, nt_sub):
        def _on_sub(event: ntcore.Event):
            # TODO: _on_sub logic
            index = self.nt_subs.index(nt_sub)
            msg_type = self.msg_types[index]

            try:
                nt_type = nt_type_dict(msg_type)
                msg = nt2msg(nt_sub.get(), msg_type.replace("/", "."))
            except KeyError:
                nt_type = "string"
                # deserialize back to ROS message
                json_msg = nt_sub.get()
                if json_msg == "":
                    return
                msg = json2msg(json_msg, msg_type.replace("/", "."))
            # self.get_logger().info(f'msg #{index}: {msg}')
            
            # publish ROS message
            self.pubs[index].publish(msg)
        return _on_sub

    def _connect_cb(self, event: ntcore.Event):
            if event.is_(ntcore.EventFlags.kConnected):
                self.get_logger().info(f"Connected to {event.data.remote_id}")
            elif event.is_(ntcore.EventFlags.kDisconnected):
                self.get_logger().info(f"Disconnected from {event.data.remote_id}")
                # self.removeDisconnectedPubs()

    # add a listener to see when new topics are published
    def _on_pub(self, event: ntcore.Event):
        if event.is_(ntcore.EventFlags.kPublish):
            # topicInfo.name is the full topic name, e.g. "/datatable/X"
            NT_name = str(event.data.name)
            indices_of_slash = list(find_all(NT_name,'/'))
            for idx_slash in indices_of_slash:
                if indices_of_slash.index(idx_slash) + 1 == len(indices_of_slash):
                    next_idx_slash = len(NT_name)
                else:
                    next_idx_slash = indices_of_slash[indices_of_slash.index(idx_slash) + 1]
                for idx in range(idx_slash + 1, next_idx_slash):
                    char = NT_name[idx]
                    if is_invalid_char(char) or ((idx == idx_slash + 1) and char.isdigit()):
                        self.get_logger().info(f'Fail to publish: {event.data.name}')
                        self.get_logger().info(f'Invalid character "{char}" found after slash at index {idx}')
                        return

            try:
                msg_type = ros_type_dict(event.data.type_str)
            except KeyError: # TODO: add support to BooleanArray and StringArray
                return
            
            self.get_logger().info(f'newly published: {event.data.name}')
            rostopic_name = '/networktable' + NT_name.replace(' ','_')
            self.sub_NT_names.append(str(event.data.name))
            self.msg_types.append(ros_type_dict(event.data.type_str))
            self.pub_rostopic_names.append(rostopic_name)
            self.update_pubs()
    
    def update_pubs(self):
        start_index = len(self.nt_subs)
        # self.get_logger().info(f'\n\n{start_index}\n')
        for nt_name in self.sub_NT_names[start_index:]:
            index = self.sub_NT_names.index(nt_name)
            msg_type = self.msg_types[index]
            try:
                nt_type = nt_type_dict(msg_type)
            except KeyError:
                nt_type = "string"
            default_value = nt_default_value(nt_type)
            self.nt_types.append(nt_type)
            self.pubs.append(self.create_publisher(findROSClass(msg_type.replace("/", ".")), self.pub_rostopic_names[index], 10))

            # create nt subscribers
            self.nt_subs.append(nt_create_topic(self.inst, nt_type, nt_name).subscribe(default_value))
            self._on_subs.append(self.make_func_on_sub(self.nt_subs[index]))
            self.valueListenersHandle.append(
                self.inst.addListener(
                    self.nt_subs[index], ntcore.EventFlags.kValueAll, self._on_subs[index]
                )
            )

    # def periodic(self):
    #     # TODO: periodic logic
    #     for nt_sub in self.nt_subs:
    #         index = self.nt_subs.index(nt_sub)
    #         msg_type = self.msg_types[index]

    #         try:
    #             nt_type = nt_type_dict(msg_type)
    #             msg = nt2msg(nt_sub.get(), msg_type.replace("/", "."))
    #         except KeyError:
    #             nt_type = "string"
    #             # deserialize back to ROS message
    #             json_msg = nt_sub.get()
    #             if json_msg == "":
    #                 continue
    #             msg = json2msg(json_msg, msg_type.replace("/", "."))
    #         # self.get_logger().info(f'msg #{index}: {msg}')
            
    #         # publish ROS message
    #         pub = self.pubs[index]
    #         pub.publish(msg)

    def create_pubs(self):
        self.update_pubs()

    def coerceSizeCheck(self):
        if len(self.sub_NT_names) == len(self.msg_types):
            if len(self.msg_types) == len(self.pub_rostopic_names):
                self.get_logger().info('Number of topics Checked!')
            else:
                raise Exception("Topic info has unmatched size!!! Please check you yaml file.")
        else:
            raise Exception("Topic info has unmatched size!!! Please check you yaml file.")
    
    def removeEmptyEntries(self):
        for nt_name in self.sub_NT_names:
            index = self.sub_NT_names.index(nt_name)
            if nt_name == "":
                self.sub_NT_names.pop(index)
                self.msg_types.pop(index)
                self.pub_rostopic_names.pop(index)

    def removeDisconnectedPubs(self):
        for nt_name in self.sub_NT_names:
            # if nt_name not in self.user_defined_sub_NT_names:
            self.get_logger().info(f'\n{len(self.sub_NT_names)}')
            index = self.sub_NT_names.index(nt_name)
            self.sub_NT_names.pop(index)
            self.msg_types.pop(index)
            self.pub_rostopic_names.pop(index)
            self.pubs[index].destroy()
            self.pubs.pop(index)
            self.nt_types.pop(index)
            self.nt_subs.pop(index)
            self._on_subs.pop(index)
            self.valueListenersHandle.pop(index)
            self.get_logger().info(f'\n{len(self.sub_NT_names)}')
        
def main(args=None):
    rclpy.init(args=args)

    node = NTClientSub()

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