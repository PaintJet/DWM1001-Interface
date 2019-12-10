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
from geometry_msgs.msg import Point

# Import DWM1001 API
from DWM1001_API import DWM1001

import time, os

class TagPublisher:

    # Constructor
    # Name of the file that points to the tag (e.g. /dev/ttyACM0)
    def __init__(self, tag_name, tag_num):
        self.tag_name = tag_name
        self.tag_num = tag_num

	os.popen("sudo chmod 777 {}".format(tag_name), "w")

        # Base topic path for messages relating to this tag
        self.topic_string = "/uwb/" + str(tag_num) 

        # Initialize DWM API object
        self.tag = DWM1001(tag_name)

        # Initialize serial communication with tag
        self.tag.begin_comm()

        # Send command to start sending distances to nodes and position of tag
        self.tag.lec()

        # Create dictionary containing the publishers for the tag
        self.anchor_publishers = {}
        # Create publisher for the position of the tag
        self.pos_publisher = rospy.Publisher(self.topic_string + "/position", Point, queue_size = 0)

    def exit(self):
        # Close serial communication to tag
        self.tag.end_comm()

    # parse_anchor_distances
    # Converts lec CSV messages into better datatypes
    # Returns the distances to anchors in a dictionary
    # Returns a list of length 3 with the pose
    def parse_anchor_distances(self, response_string):
        response_list = response_string.split(",")

        # Create dictionary to store range data from each anchor
        anchor_ranges = {}
        # Create array to store positiion data
        pos = []

        # Check if data is empty or incorrectly formatted
        # Perform validation by checking if the data begins with DIST
        msg_valid = False
        if(len(response_string) >= 6 and response_string[0:4] == "DIST"):
            response_list = response_string.split(",")

            # Extract number of anchors from which range data has been collected
            num_anchors = int(response_list[1])

            # Verify length of message is correct for reported number of tags
            # 2 is the number of elements not associated with anchors - DIST, NUM_ANCHORS
            # These two elements appear at the beginning of the message
            # Number of anchors is multiplied by 6 because each anchor has 6 associated datapoints
            # 5 is the number of elements associated with the position if it is included in the message
            # The message can take on exactly 1 or these 2 lengths
            if(len(response_list) == 2 + 6*num_anchors or len(response_list) == 2 + 6*num_anchors + 5):
                msg_valid = True

        if(msg_valid):

            for i in range(num_anchors):
                # Calculate start index of anchor data in string
                start_index = 6*i + 2

                # start_index + 1 is unique ID of UWB anchor
                # start_index + [2-4] are the x,y,z coordinates of the anchors
                # start_index + 5 is the distance to the anchor
                anchor_ranges[response_list[start_index + 1]] = {
                    "distance" : float(response_list[start_index + 5]),
                    "x" : float(response_list[start_index + 2]),
                    "y" : float(response_list[start_index + 3]),
                    "z" : float(response_list[start_index + 4])
                }

            # Get position of module if it is in response
            try:
                pos_index = response_list.index("POS")

                # Collect and parse position data from the list
                pos = [float(response_list[pos_index + 1]), float(response_list[pos_index + 2]), float(response_list[pos_index + 3])]

            except(ValueError):
                pass
        else:
            time.sleep(0.5)

        return anchor_ranges, pos



    def publish_tag_data(self):
        # Get next line from tag
        response_string = self.tag.read_response()
	
        # Get anchor distances and position
        anchors_data, pos = self.parse_anchor_distances(response_string)

        # If position is not empty
        if(pos != []):
            # Construct position message
            pos_msg = Point()
            pos_msg.x = pos[0]
            pos_msg.y = pos[1]
            pos_msg.z = pos[2]

            # Publish position
            self.pos_publisher.publish(pos_msg)

        # Iterate over all anchors in anchor distances
        for anchor_name, anchor_data in anchors_data.items():

            # Check if we have previously published a distance for the given tag
            if not (anchor_name in self.anchor_publishers):
                # Distance for this tag has not been previously published, create new publishers
                # Create new publisher
                tag_distance_topic = self.topic_string + "/anchors/{anchor_name}/distance".format(anchor_name = anchor_name)
                tag_position_topic = self.topic_string + "/anchors/{anchor_name}/position".format(anchor_name = anchor_name)
                self.anchor_publishers[anchor_name] = {
                    "distance" : rospy.Publisher(tag_distance_topic, Float64, queue_size=0),
                    "position" : rospy.Publisher(tag_position_topic, Point, queue_size=0)

                }

             # Publish to distance to topic
            self.anchor_publishers[anchor_name]["distance"].publish(anchor_data["distance"])

            # Publish position of marker to topic
            pos_msg = Point()
            pos_msg.x = anchor_data["x"]
            pos_msg.y = anchor_data["y"]
            pos_msg.z = anchor_data["z"]
            self.anchor_publishers[anchor_name]["position"].publish()





tag_names = ["/dev/ttyACM0", "/dev/ttyACM1"] # "/dev/ttyACM2"]


if __name__ == '__main__':

    # Initialize as ROS node
    rospy.init_node('uwb_tag_publisher')

    # List to store serial tag interface objects for each tag
    tags = []

    # Populate list with tag interfaces
    for i, tag_name in enumerate(tag_names):
        tags.append(TagPublisher(tag_name, i))

    # Ready to go
    rospy.loginfo("UWB Module Publisher Initialized...")
    time.sleep(2)

    r = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        # Publish tag data
        for tag in tags:
            tag.publish_tag_data()

        r.sleep()



    # Loop continuously
    rospy.spin()
