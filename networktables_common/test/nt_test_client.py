#!/usr/bin/env python3
#
# A client that publishes some synchronized values periodically

import argparse
import os
from os.path import basename
import logging
import time
from random import random, randint, choice

import ntcore

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)

    parser = argparse.ArgumentParser()
    parser.add_argument("ip", type=str, help="IP address to connect to")
    args = parser.parse_args()

    # Initialize NT4 client
    inst = ntcore.NetworkTableInstance.getDefault()

    identity = f"{basename(__file__)}-{os.getpid()}"
    inst.startClient4(identity)

    inst.setServer(args.ip)

    # publish two values
    table = inst.getTable("data")
    pub1 = table.getDoubleTopic("1").publish()
    pub2 = table.getDoubleTopic("2").publish()
    name_pub = table.getStringTopic("my_name").publish()
    bool_array_pub = table.getBooleanArrayTopic("array/bool_array").publish()
    int_array_pub = table.getIntegerArrayTopic("array/int_array").publish()
    double_array_pub = table.getDoubleArrayTopic("array/double_array").publish()
    string_array_pub = table.getStringArrayTopic("array/string_array").publish()

    i = 3

    while True:
        # These values are being published fast than the server is polling
        pub1.set(i)
        pub2.set(i + 100)
        name_pub.set("Winston")
        bool_array_pub.set([choice([True, False]), choice([True, False]), choice([True, False]), choice([True, False])])
        int_array_pub.set([randint(0, 100), randint(0, 100), randint(0, 100), randint(0, 100), randint(0, 100)])
        double_array_pub.set([random(), random(), random(), random(), random(), random()])
        possible_strings = ["alpha", "bravo", "charlie", "delta", "echo"]
        string_array_pub.set([choice(possible_strings), choice(possible_strings), choice(possible_strings)])

        i += 1

        try:
            time.sleep(0.01)
        except KeyboardInterrupt:
            print('\nProgram interrupted by user.')
            break