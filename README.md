# ridgeback_ur5_controller
[![Build Status](https://travis-ci.com/epfl-lasa/ridgeback_ur5_controller.svg?token=m4ujgeX7fDuuc9CGktAM&branch=master)](https://travis-ci.com/epfl-lasa/ridgeback_ur5_controller)

DEVEL branch: Here, we will try to extend the admittance controller.
1) the equilibrium point for the position and velocity will be received from a topic
2) the parameters of the admittance control will be dynamically reconfigurable.




This package implements an admittance controller on the ridgeback+UR5 platform. It also provides a cartesian velocity controller (ros control) for the UR5 arm. 

---

## compliation and build

Clone the repository intor your catkin source directory
```bash
$ cd ~/catkin_ws/src
$ git clone git@github.com:epfl-lasa/ridgeback_ur5_controller.git
```

Get the source dependencies using wstool
```bash
$ wstool init
$ wstool merge ridgeback_ur5_controller/dependencies.rosinstall
$ wstool up
```
Get the package dependencies using rosdep
```bash
$ rosdep install -y --from-paths src --ignore-src --rosdistro indigo
```
You also need the following ros packages
```bash
$ sudo apt-get install ros-indigo-ridgeback-*
$ sudo apt-get install ros-indigo-universal-robot
```
if you are getting error for broken packages (most probably due to a wrong version of gazebo), you can use 'aptitude' instead of 'apt-get' which propose a solution and resolve the conflict. 


Finally complie
```bash
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
$ catkin_make
```
* you might need the source the bash file and compie again if the first compliation could not find some of in house dependencies.

## Control Architecture

<object data="http://yoursite.com/the.pdf" type="application/pdf" width="700px" height="700px">
    <embed src="fig_control_schematics.pdf">
    </embed>
</object>



---


## Running the controller


To bring up the robot in simulation run
```
roslaunch cpr_bringup cpr_bringup.launch
roslaunch admittance_control admittance_controller.launch
```
For the real robot launch on the CPR main PC run
```
roslaunch cpr_bringup cpr_bringup.launch sim:=false
roslaunch admittance_control admittance_controller_real.launch
```
