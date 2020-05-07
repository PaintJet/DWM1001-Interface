# DWM1001-Interface
This repository contains a ROS interface for the DecaWave DWM1001 Ultra-Wide Band (UWB) modules. The package includes two useful tools: a Python interface for the UWB modules to communicate with them over Serial and a ROS interface to publish the localization data. The `dwm1001-tag` package contains the interface to read from tags. The anchor interface to communicate with and configure anchors is in the development pipeline.

## Usage
### Setup DWM1001 network
To use this package, a network for UWB modules must first be configured using the DRTLS mobile app developed by DecaWave (Note: Once the anchor interface is created, this setup process should be possible through the API in this codebase).

#### Anchor Setup
All of the anchors should be set to anchor mode in the app. Exactly one anchor must be selected as the initiator. The UWB mode should be set to active and the position of the anchor in the room must be entered.

#### Tag Setup
All of the tags should be set to tag mode. The following settings should be set for the tags
```
UWB Mode: Active
BLE: Enabled
Responsive Mode: Enabled
Location Engine: Enabled
Stationary Detection: Enabled
Normal Update Rate: 100 ms/10 Hz
Stationary Update Rate: 100 ms/10 Hz
```

Each of the tags will show up as `/dev/TTYACM[X]` by default on the computer they are plugged into. However, the order in which they are listed will change depending on how they are discovered as they are started up. Therefore, the script `create_udev_rules.sh` should be used to assign unique names to each device. When running the script, you will have to manually change the desired device name in the script.

## Custom ROS message
To make the message communcation to the particle filter more efficient, we defined a custom ROS message that carries the range measurements from all the anchors. The message structure is as follows

#### UWBReading
string anchor_id - Unique ID of the anchor
geometry_msgs/Point anchor_position - 2D coordinate of the anchor
float32 anchor_range - Distance measurement to the anchor

#### UWBReadingArray
Header header
dwm1001_tag/UWBReading[] readings - Array of UWBReadings
