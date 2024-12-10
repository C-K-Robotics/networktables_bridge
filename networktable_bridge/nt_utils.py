#!/usr/bin/env python3

import json
import yaml
import importlib

import numpy as np
import ntcore

from rosidl_runtime_py import set_message_fields

def msg2json(msg):
    ''' 
    Convert a ROS message to JSON format

    :param msg: original ROS msg
    :return: returns with a tuple of (json_string, msg_type)
    '''
    msg_string = str(msg)
    msg_type = msg_string[:msg_string.find('(')]
    return json.dumps(todict(msg)), msg_type

def json2msg(json_string: str, msg_type: str):
    ''' 
    Convert the encoded JSON string back to desired ROS message

    :param json_string: the encoded data in JSON format
    :param msg_type: the type of the ROS message
    :return: returns with the restored ROS message
    '''
    var_class = findROSClass(msg_type)
    restored_msg = var_class()
    set_message_fields(restored_msg, json.loads(json_string))
    return restored_msg

def nt2msg(nt_value, msg_type: str):
    ''' 
    Convert the NT value directly to desired ROS message

    :param nt_value: the NT value
    :param msg_type: the type of the ROS message
    :return: returns with the restored ROS message
    '''
    var_class = findROSClass(msg_type)
    restored_msg = var_class()
    _dict = {"data": nt_value}
    set_message_fields(restored_msg, _dict)
    return restored_msg

def findROSClass(msg_type: str):
    ''' 
    Find the class of the given ROS message type

    :param msg_type: the type of the ROS message
    :return: returns with the class
    '''
    delimiter_index = msg_type.rfind('.')
    var_class = importlib.import_module(msg_type[:delimiter_index]).__getattribute__(msg_type[delimiter_index + 1:])
    return var_class

def todict(obj):        
    if isinstance(obj, dict):
        return dict((key.lstrip("_"), todict(val)) for key, val in obj.items())
    elif hasattr(obj, "_ast"):
        return todict(obj._ast())
    elif hasattr(obj, "__iter__") and not isinstance(obj, str):
        return [todict(v) for v in obj]
    elif hasattr(obj, '__dict__'):
        return todict(vars(obj))
    elif hasattr(obj, '__slots__'):
        return todict(dict((name, getattr(obj, name)) for name in getattr(obj, '__slots__')))
    return obj

nt_dict = {
    "std_msgs/msg/Bool":"Boolean",
    "std_msgs/msg/Float64MultiArray":"DoubleArray",
    "std_msgs/msg/Float64":"Double",
    "std_msgs/msg/Float32MultiArray":"FloatArray",
    "std_msgs/msg/Float32":"Float",
    "std_msgs/msg/Int32MultiArray":"IntegerArray",
    "std_msgs/msg/Int32":"Integer", 
    "std_msgs/msg/String":"String"
}

# ros_dict = dict()
# for key, value in nt_dict.items():
#     ros_dict[value].append(key)         

def nt_type_dict(ros_type:str):
    return nt_dict[ros_type]

# def ros_type_dict(nt_type:str):
#     return ros_dict[nt_type]

def nt_create_topic(inst:ntcore.NetworkTableInstance, topic_type:str, topic_name:str):
    name = topic_name[(topic_name.rfind("/")) + 1:]
    table_name = topic_name[:(topic_name.rfind("/"))]
    table = inst.getTable(table_name)

    if topic_type == "BooleanArray":
        return table.getBooleanArrayTopic(name)
    elif topic_type == "Boolean":
        return table.getBooleanTopic(name)

    elif topic_type == "DoubleArray":
        return table.getDoubleArrayTopic(name)
    elif topic_type == "Double":
        return table.getDoubleTopic(name)

    elif topic_type == "FloatArray":
        return table.getFloatArrayTopic(name)
    elif topic_type == "Float":
        return table.getFloatTopic(name)
        
    elif topic_type == "IntegerArray":
        return table.getIntegerArrayTopic(name)
    elif topic_type == "Integer":
        return table.getIntegerTopic(name)
    
    elif topic_type == "StringArray":
        return table.getStringArrayTopic(name)
    elif topic_type == "String":
        return table.getStringTopic(name)

def nt_default_value(topic_type:str):
    if topic_type == "BooleanArray":
        return [False]
    elif topic_type == "Boolean":
        return False

    elif topic_type == "DoubleArray":
        return [0.0]
    elif topic_type == "Double":
        return 0.0

    elif topic_type == "FloatArray":
        return [0.0]
    elif topic_type == "Float":
        return 0.0
        
    elif topic_type == "IntegerArray":
        return [0]
    elif topic_type == "Integer":
        return 0
    
    elif topic_type == "StringArray":
        return [""]
    elif topic_type == "String":
        return ""