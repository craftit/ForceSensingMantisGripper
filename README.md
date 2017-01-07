# Force Sensingi Mantis Gripper
Force sensing for the mantis gripper, with ROS integration and adaptor for the UR5

## The Mantis Gripper

This project takes the gripper for Andreas Hölldorfer's gripper for his Mantis robot arm and adds force sensing and ROS integration. It also contains a URDF model, collision meshes, and visual meshes for integration with ROS MoveIt! 

The information on the gripper can be found at: http://chaozlabs.blogspot.co.uk/2016/04/mantis-robot-arm-part-1-gripper.html

The parts for the gripper can be downloaded from thingiverse: https://www.thingiverse.com/thing:1480408

## Outstanding issues

- The servo motor can still overhead if left gripping an object too long. This is particularly a problem if you print the gripper parts in PLA
- Ideally it would put on a custom PCB or use an existing smart servo controller.

## License

Except for files based on the Mantis Gripper which are creative commonds, 
this project is distributed under the MIT License.

## Files


- CAD - 3D files for the parts, created with Freecad 0.16
- Electronics - Description of the electronics used for the controller.
- src - source code for the project
- src/Firmware - Code to be run on the Arduino.
- src/ROS - ROS node to talk to the gripper.
- media - Pictures of the build.


  
 
