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
### Publish Location Data
Currently, there is no launch file to launch the publisher node for the tag data (this is in the pipeline). In the tag publisher file, the list of USB devices representing the tags must be entered. To publish the tag data, simply run the `tag_publisher.py` script. The script will publish the following data for each tag:

* Distance to each anchor
* Location estimate from the built-in location engine
* Stored coordinates of each anchor
