#!/usr/bin/env python

# ----------------------------------------------- #
# This script reads data from the DWM 1001 modules over UART (USB) and publishes it as ROS messages
# To read data over UART, the shell mode of the DWM1001 is currently used
# Future versions of this software may switch over to the generic API
# This version of the software also does not configure the UWB network and the operating mode of the DWM1001 modules
# Configuration must be completed using the Android app
# ----------------------------------------------- #

# import required packages
import rospy
from std_msgs.msg import Float64

import serial
import time

class TagSerialInterface:

    # Constructor
    # Name of the file that points to the tag (e.g. /dev/ttyACM0)
    def __init__(self, tag_name):
        self.tag_name = tag_name


    # Callback function for joystick controls
    def begin_comm(self):
        # Create Serial object
        self.ser = serial.Serial(self.tag_name, 115200, timeout=0)

        # Writing two enters characters to tag to enter shell mode
        self.ser.write(b'\r\r')

        # Close serial port in case it was not properly closed during last run
        self.ser.close()

        time.sleep(1)

        # open serial port
        self.ser.open()

        # Clear input buffer
        self.ser.reset_input_buffer()

    def end_comm(self):
        # Write quit to tag to exit UWB module shell mode
        self.ser.write(b'quit')

        # Clear input buffer
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        self.ser.close()

        time.sleep(0.5)



    def read_anchor_distances(self):
        # Reset input and output buffers
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        # Write lec command to serial buffer to request tag distances from module
        msg = b'lec\r'
        self.ser.write(msg)


        # time.sleep(0.5)

        #TODO include timeout
        response = self.ser.readline()

        # Wait until response with data is received
        while not (response != "dwm> ".encode() and response.decode().strip() != msg.decode().strip() and response != "".encode()):
            print(response.decode().strip())
            response = self.ser.readline()
            time.sleep(0.1)

        print(response.decode().strip())
        #TODO Handle

        self.ser.write(b'lec\r')
        time.sleep(0.01)

        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        #TODO return data in array of dictionaries
        # return data

        # bl = 0

        # while bl < 3:
        #     response = ser.readline()
        #     if response == "".encode():
        #         bl = bl+1
        #     time.sleep(0.2)
        #     if :
        #         # do whatever you want with this response
        #         print(response.decode().strip())

    # def read_tag_position(self):

    # def end_comm(self):




tagNames = ["/dev/ttyACM0"]#, "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3"]


if __name__ == '__main__':

    # # Initialize as ROS node
    # # rospy.init_node('uwb_tag_publisher')

    # List to store serial tag interface objects for each tag
    tagInterfaces = []

    # Populate list with tag interfaces
    for tag in tagNames:
        tagInterfaces.append(TagSerialInterface(tag))

    # Initialize UART communication for all tags
    for tag in tagInterfaces:
        tag.begin_comm()
        tag.read_anchor_distances()
        tag.end_comm()



    # # Create list of dictionaries for publishers for all tags
    # # Each dictionary stores the publsihers related to each tag
    # tagPublishers = [{} for tag in tagNames]

    # # Ready to go
    # rospy.loginfo("UWB Module Publisher Initialized...")

    # while not rospy.is_shutdown():
    #     # Publish distances to each anchor for each tag
    #     for tag_num, (tag, publishers) in enumerate(zip(tagInterfaces, tagPublishers)):
    #         # Get the anchor distances
    #         anchor_distances = tag.read_anchor_distances()

    #         # Iterate over all anchors in anchor distances
    #         for anchor_name, anchor_distance in anchor_distances.items():

    #             # Check if we have previously published a distance for the given tag
    #             if anchor_name in publishers:
    #                 publishers[anchor_name].publish(anchor_distance)
    #             else:
    #                 tag_topic = "/uwb/tag_{tag_num}/{anchor_name}/distance".format(
    #                     tag_num = tag_num,
    #                     anchor_number = anchor_name
    #                 )
    #                 publishers[anchor_name] = rospy.Publisher(tag_topic, Float64, queue_size=0)


    #     # Loop continuously
    #     rospy.spin()