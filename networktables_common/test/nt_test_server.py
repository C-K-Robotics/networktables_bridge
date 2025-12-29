#!/usr/bin/env python3
#
# A server that reads from the subscription
#

import logging
import time
import netifaces as ni
from random import random, randint

import ntcore

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)

    ip = ni.ifaddresses('lo')[ni.AF_INET][0]['addr']
    # ip = "169.254.0.12" # PC: 169.254.212.29

    # initialize networktables server (on a robot this is already done)
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startServer(listen_address=ip)

    # Initialize two subscriptions
    table = inst.getTable("data")

    # only keep the latest value for this topic
    sub1 = table.getDoubleTopic("1").subscribe(-1.0)

    # keep the last 10 values for this topic
    sub2 = table.getDoubleTopic("2").subscribe(
        -2.0, ntcore.PubSubOptions(pollStorage=10)
    )

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

    nt_clients_table = sys_table.getSubTable("NTClients")
    last_nt_remote_ids = set()

    # Periodically read from them
    # - note sub1 only has 1 value, but sub2 sometimes has more than 1
    while True:
        print("---", ntcore._now())
        print("/data/1:", sub1.readQueue())
        print("/data/2:", sub2.readQueue())

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
        rsl_state_pub.set(randint(0, 1) == 0)    # Example RSL state
        sys_time_valid_pub.set(True)  # Example system time valid status

        nt_conns = inst.getConnections()
        nt_remote_ids = set()
        for conn in nt_conns:
            if conn.remote_id in last_nt_remote_ids:
                last_nt_remote_ids.remove(conn.remote_id)
            nt_remote_ids.add(conn.remote_id)
            nt_client_table = nt_clients_table.getSubTable(conn.remote_id)

            nt_client_table.getBooleanTopic("Connected").publish().set(True)
            nt_client_table.getStringTopic("IPAddress").publish().set(conn.remote_ip)
            nt_client_table.getIntegerTopic("RemotePort").publish().set(conn.remote_port)
            nt_client_table.getIntegerTopic("ProtocolVersion").publish().set(conn.protocol_version)

        for stale_id in last_nt_remote_ids:
            nt_client_table = nt_clients_table.getSubTable(stale_id)
            nt_client_table.getBooleanTopic("Connected").publish().set(False)

        time.sleep(0.1)