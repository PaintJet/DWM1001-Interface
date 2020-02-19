    # This script reads data from the DWM 1001 modules over UART (USB) and publishes it as ROS messages
# To read data over UART, the shell mode of the DWM1001 is currently used
# Future versions of this software may switch over to the generic API
# This version of the software also does not configure the UWB network and the operating mode of the DWM1001 modules
# Configuration must be completed using the Android app
# ----------------------------------------------- #

# import required packages
import rospy

import serial
import time

class DWM1001:

    # Constructor
    # tag_name - Name of the file that points to the tag (e.g. /dev/ttyACM0)
    def __init__(self, tag_name):
        self.tag_name = tag_name

    def __del__(self):
        self.end_comm()

    # read_response
    # Reads a message sent by UWB module
    # return : string containing a line (ended by \n) of the serial output of DWM1001 module
    def read_response(self):
        response = self.ser.readline()
	print(response)
        return response.decode().strip()

    def begin_comm(self):
	print("Initializing Communication with tag {}".format(self.tag_name))

        # Create Serial object
        self.ser = serial.Serial(
	     port = self.tag_name,
             baudrate =  115200,
             parity = serial.PARITY_ODD,
             stopbits = serial.STOPBITS_TWO,
             bytesize = serial.SEVENBITS)

        # Close serial port in case it was not properly closed during last run
        self.ser.close()

        time.sleep(2)

        # open serial port
        self.ser.open()

	time.sleep(2)

        # Clear serial buffer
	self.ser.reset_input_buffer()
	self.ser.reset_output_buffer()

        # Writing two enters characters to tag to enter shell mode
	response = None
	while(response != "dwm>"):
		self.double_enter()
		time.sleep(0.2)
		response = self.read_response()
	time.sleep(0.2)

	print("Successfully Initialized tag {}".format(self.tag_name))

        # Clear input buffer
        self.ser.reset_input_buffer()
	self.ser.reset_output_buffer()

    def end_comm(self):
        # Reset tag to stop data transmission and exit shell mode
        self.reset()

        time.sleep(0.5)

        # Clear input buffer
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        self.ser.close()

        time.sleep(0.5)

    #################################### API FUNCTION DEFINITIONS ####################################
    # The following functions are defined to send shell commands as specified by the API

    # ASCII char for double enter
    def double_enter(self):
        self.ser.write(b'\r\r')

    # ASCII char for single enter
    def enter(self):
        self.ser.write(b'\r')

    # Display help
    def help(self):
        self.ser.write(b'?\r')

    # Exit shell mode
    def quit(self):
        self.ser.write(b'quit\r')

    # Clears GPIO pin
    def gc(self):
        self.ser.write(b'gc\r')

    # Reads GPIO pin level
    def gg(self):
        self.ser.write(b'gg\r')

    # Sets GPIO as output and sets its value
    def gs(self):
        self.ser.write(b'gs\r')

    # Toggle GPIO(must be an output)
    def gt(self):
        self.ser.write(b'gt\r')

    # Show free memory on the heap
    def f(self):
        self.ser.write(b'f\r')

    # Show info about running threads
    def ps(self):
        self.ser.write(b'ps\r')

    # Show power managements tasks. IDL means that task is idle. USE means that task is allocated in the power management
    def pms(self):
        self.ser.write(b'pms\r')

    # reset the dev board
    def reset(self):
        self.ser.write(b'reset\r')

    # Show device uptime
    def ut(self):
        self.ser.write(b'ut\r')

    # Factory reset
    def frst(self):
        self.ser.write(b'frst\r')

    # General purpose I2C/TWI read
    def twi(self):
        self.ser.write(b'twi\r')

    # Read ACC device ID
    def aid(self):
        self.ser.write(b'aid\r')

    # Rad ACC values
    def av(self):
        self.ser.write(b'av\r')

    # Show distances to ranging anchors and the position if location engine is enabled
    def les(self):
        self.ser.write(b'les\r')

    # Show measurement and position in CSV format
    def lec(self):
        self.ser.write(b'lec\r')

    # Show position in CSV format.Sending this command multiple times will turn on/off this functionality.
    def lep(self):
        self.ser.write(b'lep\r')

    # System Info
    def si(self):
        self.ser.write(b'si\r')

    # Get node mode info
    def nmg(self):
        self.ser.write(b'nmg\r')

    # Enable passive offline option and resets the node
    def nmo(self):
        self.ser.write(b'nmo\r')

    # Enable active offline option and resets the node.
    def nmp(self):
        self.ser.write(b'nmp\r')

    # Configures node to as anchor, active and reset the node.
    def nma(self):
        self.ser.write(b'nma\r')

    # Configures node to as anchor initiator, active and reset the node.
    def nmi(self):
        self.ser.write(b'nmi\r')

    # Configures node to as tag, active and reset the node
    def nmt(self):
        self.ser.write(b'nmt\r')

    # Configures node to as tag, active, low power and resets the node.
    def nmtl(self):
        self.ser.write(b'nmtl\r')

    # Toggle UWB bandwidth / tx power compensation.
    def bpc(self):
        self.ser.write(b'bpc\r')

    # Show anchor list
    def la(self):
        self.ser.write(b'la\r')

    # Display statistics
    def stg(self):
        self.ser.write(b'stg\r')

    # Clears statistics
    def stc(self):
        self.ser.write(b'stc\r')

    # Parses given tlv frame, see section 4 for valid TLV commands
    def tlv(self):
        self.ser.write(b'tlv\r')

    # Set position update rate. See section 4.3.3 for more detail.
    def aurs(self):
        self.ser.write(b'aurs\r')

    # Get position update rate. See section 4.3.4 for more details
    def aurg(self):
        self.ser.write(b'aurg\r')

    # Get position of the node.See section 3.4.2 for more detail
    def apg(self):
        self.ser.write(b'apg\r')

    # Set position of the node. See section 3.4.2for more detail
    def aps(self):
        self.ser.write(b'aps\r')

    # Configures node as anchor with given options
    def acas(self):
        self.ser.write(b'acas\r')

    # Configures node as tag with given options
    def acts(self):
        self.ser.write(b'acts\r')

