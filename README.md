# Map Multiplexer
### About
The Map Multilpexer or Map Mux for short is a ROS node supports multi-level robots and aids in their navigation and localisation when they traverse from one floor to another via lifts or perhaps even escalators.

### What is it for?
The Map Mux works in conjunction with [RMF](https://github.com/open-rmf/rmf), [Free Fleet](https://github.com/open-rmf/free_fleet) and [Multimap server](https://github.com/RobotnikAutomation/multimap_server).

While RMF Supports the management of robots for multiple levels, robots running in ROS1 do not have any "off-the-shelf" packages or features that helps the robot change its map to that of the new level and help it to re-localise on the new floor after it has exited the lift.

### How it works
![](media/mulitmap_server-map_mux_rosgraph.svg)
In short the Map Mux subscribes to several map topics as well as the Map Request topic. Based on the Map Request that it has received, the Map Mux will that publish the requested map to the `map` topic which is used by the robot's navigation stack.

This trigger will be started by the RMF fleet adapter. *(yet to be implemented)* The RMF will send a map request to the Free Fleet(FF) Server. The FF Server will then publish this as a DDS Topic which the FF Client will pick up and publish as a ROS 1 topic with the topic name `map_request`. The Map Mux who is subscribed to the `map_request` topic will then publish the requested map and send and initial pose estimate based on the lift that the robot is taking.

### Configuring the Map Mux's map subscription
The Map Mux subscribes to several maps published by the multimap server depending on the number of levels you want it to support. the number of floors can be configured in yaml files that are stored in the config directory.



