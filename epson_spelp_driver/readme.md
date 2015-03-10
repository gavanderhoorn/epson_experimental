# Epson SPEL+ driver

This package contains an Epson RC+ project that implements a [ROS-Industrial
driver specification] [rosi_spec] compatible state and trajectory server.


## Compatibility

The `epson_spelp_driver` project was created with Epson RC+ v7.0.5.

Compiler compatibility was set to *default* on the project properties
pages.


## Setup

### Project setup in RC+

Copy the directory structure under the `spel+` directory to the `projects`
directory of your Epson RC+ installation. Open the `epson_spelp_driver`
project from the `ros_industrial` project folder.


### Controller setup in RC+

Make sure to check the *PC to Controller Communications* and *System
Configuration* screens and use the correct settings for the target
controller and manipulator.


### Building

Use the *Build Project* toolbar button, press `ctrl+b` or choose
*Project->Build* from the menu. No errors or warnings should be reported,
and the project should be copied to the controller.



[rosi_spec]: http://wiki.ros.org/Industrial/Industrial_Robot_Driver_Spec
             "ROS-Industrial Robot Driver Specification (Final), online, retrieved 2015-03-10"
