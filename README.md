# ridgeback_ur5_controller
[![Build Status](https://travis-ci.com/epfl-lasa/ridgeback_ur5_controller.svg?token=m4ujgeX7fDuuc9CGktAM&branch=master)](https://travis-ci.com/epfl-lasa/ridgeback_ur5_controller)

This package implements an admittance controller on the ridgeback+UR5 platform. It also provides a cartesian velocity controller (ros control) for the UR5 arm. 

To bring up the robot in simulation run
```
roslaunch cpr_bringup cpr_bringup.launch
```
For the real robot launch on the CPR main PC  
```
roslaunch cpr_bringup cpr_bringup.launch sim:=false
```

To launch the admittance controller just run
```
roslaunch admittance_control admittance_controller.launch
```
