#!/usr/bin/env python
# Author: Andrea Colucci
# Master thesis at CIM 4.0 -- Torino 2021
# This code reads local position coordinates from an UWB (DWM1001c - Qorvo)
# and then publishes it onto the topic called "/mavros/fake_gps/mocap/tf"
# created from a mavros plugin in order to be able to send gps coordinates
# to the onboard Pixhawk and be able to fly in GNNS-denied environments.

import rospy
import time
import serial
from pkg.msg import TransformStamped

def talker():
    DWM = serial.Serial(port="/dev/ttyACM0", baudrate=115200)
    print("Connected to " + DWM.name)
    DWM.write("\r\r".encode())  # Initializes the tag
    time.sleep(1)
    DWM.write("lec\r".encode())  # Standard message to read the coordinates
    time.sleep(1)
    pub = rospy.Publisher('/mavros/fake_gps/mocap/tf', TransformStamped, queue_size=10)  # create a ROS publisher
    rospy.init_node('talker_gps_fix', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz -- Frequency of the GPS
    while not rospy.is_shutdown():
        try:
            line = DWM.readline()
            if (line):
                if (len(line) >= 140) and ("POS" in line):  # evaluate whether it reads correctly or not
                    parse = line.decode().split(",")  #
                    x_pos = float(parse[parse.index("POS") + 1])  #
                    y_pos = float(parse[parse.index("POS") + 2])  #
                    z_pos = float(parse[parse.index("POS") + 3])  #
                    pos = (x_pos, y_pos, z_pos)
                    fix_obj = TransformStamped()
                    fix_obj.header.frame_id = "map"
                    fix_obj.child_frame_id = "fix"
                    fix_obj.transform.translation.x = x_pos
                    fix_obj.transform.translation.y = y_pos
                    fix_obj.transform.translation.z = z_pos
                    fix_obj.transform.rotation.x = 0
                    fix_obj.transform.rotation.y = 0
                    fix_obj.transform.rotation.z = 0
                    fix_obj.transform.rotation.w = 0
                    pub.publish(fix_obj)
                    rate.sleep()
                    print(pos)
                else:
                    print("Position not calculated: ", line.decode())
        except Exception as ex:
            print(ex)
            break
    DWM.write("\r".encode())
    DWM.close()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
