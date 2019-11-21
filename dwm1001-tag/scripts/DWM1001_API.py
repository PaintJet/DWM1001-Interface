class DWM1001Tag:
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

class TagPublisher:

    # Constructor
    # tag_name - Name of the file that points to the tag (e.g. /dev/ttyACM0)
    def __init__(self, tag_name):
        self.tag_name = tag_name

    # read_response
    # Reads a message sent by UWB module
    # return : string containing a line (ended by \n) of the serial output of DWM1001 module
    def read_response():
        response = self.ser.readline()

        return response.decode().strip()

    #################################### API FUNCTION DEFINITIONS ####################################
    # The following functions are defined to send shell commands as specified by the API

    # ASCII char for double enter
    def double_enter(self):
        self.ser.write(b'\r\r')

    # ASCII char for single enter
    def double_enter(self):
        self.ser.write(b'\r')

    # Display help
    def help(self):
        self.ser.write(b'?')

    # Exit shell mode
    def quit(self):
        self.ser.write(b'quit')

    # Clears GPIO pin
    def gc(self):
        self.ser.write(b'gc')

    # Reads GPIO pin level
    def gg(self):
        self.ser.write(b'gg')

    # Sets GPIO as output and sets its value
    def gs(self):
        self.ser.write(b'gs')

    # Toggle GPIO(must be an output)
    def gt(self):
        self.ser.write(b'gt')

    # Show free memory on the heap
    def f(self):
        self.ser.write(b'f')

    # Show info about running threads
    def ps(self):
        self.ser.write(b'ps')

    # Show power managements tasks. IDL means that task is idle. USE means that task is allocated in the power management
    def pms(self):
        self.ser.write(b'pms')

    # reset the dev board
    def reset(self):
        self.ser.write(b'reset')

    # Show device uptime
    def ut(self):
        self.ser.write(b'ut')

    # Factory reset
    def frst(self):
        self.ser.write(b'frst')

    # General purpose I2C/TWI read
    def twi(self):
        self.ser.write(b'twi')

    # Read ACC device ID
    def aid(self):
        self.ser.write(b'aid')

    # Rad ACC values
    def av(self):
        self.ser.write(b'av')

    # Show distances to ranging anchors and the position if location engine is enabled
    def les(self):
        self.ser.write(b'les')

    # Show measurement and position in CSV format
    def lec(self):
        self.ser.write(b'lec')

    # Show position in CSV format.Sending this command multiple times will turn on/off this functionality.
    def lep(self):
        self.ser.write(b'lep')

    # System Info
    def si(self):
        self.ser.write(b'si')

    # Get node mode info
    def nmg(self):
        self.ser.write(b'nmg')

    # Enable passive offline option and resets the node
    def nmo(self):
        self.ser.write(b'nmo')

    # Enable active offline option and resets the node.
    def nmp(self):
        self.ser.write(b'nmp')

    # Configures node to as anchor, active and reset the node.
    def nma(self):
        self.ser.write(b'nma')

    # Configures node to as anchor initiator, active and reset the node.
    def nmi(self):
        self.ser.write(b'nmi')

    # Configures node to as tag, active and reset the node
    def nmt(self):
        self.ser.write(b'nmt')

    # Configures node to as tag, active, low power and resets the node.
    def nmtl(self):
        self.ser.write(b'nmtl')

    # Toggle UWB bandwidth / tx power compensation.
    def bpc(self):
        self.ser.write(b'bpc')

    # Show anchor list
    def la(self):
        self.ser.write(b'la')

    # Display statistics
    def stg(self):
        self.ser.write(b'stg')

    # Clears statistics
    def stc(self):
        self.ser.write(b'stc')

    # Parses given tlv frame, see section 4 for valid TLV commands
    def tlv(self):
        self.ser.write(b'tlv')

    # Set position update rate. See section 4.3.3 for more detail.
    def aurs(self):
        self.ser.write(b'aurs')

    # Get position update rate. See section 4.3.4 for more details
    def aurg(self):
        self.ser.write(b'aurg')

    # Get position of the node.See section 3.4.2 for more detail
    def apg(self):
        self.ser.write(b'apg')

    # Set position of the node. See section 3.4.2for more detail
    def aps(self):
        self.ser.write(b'aps')

    # Configures node as anchor with given options
    def acas(self):
        self.ser.write(b'acas')

    # Configures node as tag with given options
    def acts(self):
        self.ser.write(b'acts')

