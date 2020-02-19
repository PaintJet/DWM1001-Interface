#!/bin/bash

echo "remap the device serial port(ttyACMX) to  uwb_5, uwb_6, uwb_7"
echo "uwb_5 usb connection as /dev/uwb_5, check it using the command : ls -l /dev|grep ttyACM"
echo "start copy uwb.rules to  /etc/udev/rules.d/"
echo "`rospack find dwm1001-tag`/udev_scripts/uwb.rules"
sudo cp `rospack find dwm1001-tag`/udev_scripts/uwb.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish "
