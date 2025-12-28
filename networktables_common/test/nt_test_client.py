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

    sys_table = inst.getTable("AdvantageKit/SystemStats")
    team_number_pub = sys_table.getIntegerTopic("TeamNumber").publish()
    bat_voltage_pub = sys_table.getDoubleTopic("BatteryVoltage").publish()
    bat_current_pub = sys_table.getDoubleTopic("BatteryCurrent").publish()
    v3v3_pub = sys_table.getDoubleTopic("3v3Rail/Voltage").publish()
    c3v3_pub = sys_table.getDoubleTopic("3v3Rail/Current").publish()
    v5v_pub = sys_table.getDoubleTopic("5vRail/Voltage").publish()
    c5v_pub = sys_table.getDoubleTopic("5vRail/Current").publish()
    v6v_pub = sys_table.getDoubleTopic("6vRail/Voltage").publish()
    c6v_pub = sys_table.getDoubleTopic("6vRail/Current").publish()
    cpu_temp_pub = sys_table.getDoubleTopic("CPUTempCelsius").publish()
    canbus_util_pub = sys_table.getFloatTopic("CANBus/Utilization").publish()
    sys_active_pub = sys_table.getBooleanTopic("SystemActive").publish()
    rsl_state_pub = sys_table.getBooleanTopic("RSLState").publish()
    sys_time_valid_pub = sys_table.getBooleanTopic("SystemTimeValid").publish()

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

        # Example system stats
        team_number_pub.set(8020)  # Example team number
        bat_voltage_pub.set(12.5 + 0.5*random())  # Example battery voltage
        bat_current_pub.set(0.2 + 3.0*random())   # Example battery current
        v3v3_pub.set(3.3 + 0.1*random())          # Example 3.3V rail voltage
        c3v3_pub.set(0.1 + 1.0*random())          # Example 3.3V rail current
        v5v_pub.set(5.0 + 0.1*random())           # Example 5V rail voltage
        c5v_pub.set(0.2 + 2.0*random())           # Example 5V rail current
        v6v_pub.set(6.0 + 0.1*random())           # Example 6V rail voltage
        c6v_pub.set(0.3 + 3.0*random())           # Example 6V rail current
        cpu_temp_pub.set(40.0 + 10.0*random())    # Example CPU temperature
        canbus_util_pub.set(0.5 + 0.5*random())   # Example CAN bus utilization
        sys_active_pub.set(True)   # Example system active status
        rsl_state_pub.set(i%2 == 0)    # Example RSL state
        sys_time_valid_pub.set(True)  # Example system time valid status

        i += 1

        try:
            time.sleep(0.01)
        except KeyboardInterrupt:
            print('\nProgram interrupted by user.')
            break