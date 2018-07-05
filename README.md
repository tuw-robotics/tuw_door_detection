# tuw_door_detection

Package containing a door detector operating on laser scans.

# tuw_object_publisher

Package supporting the static publishing of various Objects.
This package constructs different types of tuw_geometry_msgs::ObjectDetection and publishes them.
The objects position and orientation as well as their shape parameters are read from a csv file in the files/ folder. Currently cones and doors are supported objects, but the framework can be easily extended.

## Extension to other object classes:
1. Modify the config.yaml to include the new csv file describing the objects location and other shape parameters.
2. Create a new C++ class that inherits from BasePubObject and implement the generateMsg() method. Additionally you can also override the read method if you want to change this behaviour.
3. In the object_facade.h include a construction mechanism for the new class, the objects are then automatically handled by the framework supposing the yaml file has been adapted correctly.
