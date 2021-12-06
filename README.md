# Final_Project_Mixotics Package: Sir Mix-a-lot, the cafe bot!

Authors and Maintainers: 
- **Ryan King-Shepard (RKS)**
- **Marshall Johnson**
- **Matt Short**
- **Eric Codrea**
- **Pranav Kaarthik**  

## **Description**
This package enables the Franka robot to create and serve a variety of drinks.

## **User Guide (temp)**

1. Download the `mixotics.rosinstall` file into the /src directory of the catkin workspace. This file will install the ros dependencies required to run this project.
2. Use `wstool init` and `wstool merge mixotics.rosinstall` to initialize the workspace and merge the rosinstall file with the workspace. Run `wstool update` to ensure latest versions of all packages are installed.
3. Source and `catkin_make` the workspace (source it as `source ~/$(workspace name)/devel_isolated/setup.bash`).
4. To connect to the robot, plug in the ethernet cord from the robots workstation PC into the ethernet port on the users computer.
5. Run `ping robot.franka.de` to ensure that the connection is successful.
6. Open a web browser and navigate to https://robot.franka.de . Log into the website using the credentials for the robot. This interface enables us to lock/unlock the robot. At this point, ensure that the lights on the robot glow yellow.
7. If another user was using the robot prior to connecting, the user will be prompted to request control.
    a. Press request control
    b. Press enforce
    c. When the interfaces prompts the user to, click the button with a circle that is on top of the robot.
8. Make sure the robot is disabled by pressing down on the enabling device. On the website interface, from the menu on the right click on the unlock button. The lights should change from green to white. This places the robot in free move mode, and the user can manually move the robot by pressing on the two buttons on the end of the arm.
9. Open the hamburger menu on the top right, and select Activate Franka Control Interface (FCI).
10. To have control of the Emika Franka robot, open two terminals and source the workspace in both.
    a. In the first terminal, run `roscore`.
    b. In the second terminal, run `export ROS_MASTER_URI=http://$(hostname):11311` where hostname is the name of the users computer. After that, run `ssh -oSendEnv=ROS_MASTER_URI student@station`.
11. We can now start the franka_ros controller. To do so, run `roslaunch franka_control franka_control.launch robot_ip:=robot.franka.de`. Keep this terminal open - if this process stops running, it needs to be restarted to be able to run the robot.
12. Now that the connection to the robot is setup and the franka_ros controllers are running, run the launchfile `roslaunch panda_moveit_config panda_control_moveit_rviz.launch launch_franka_control:=false robot_ip:=robot.franka.de` to run moveit and rviz.
13. In another terminal, run `roslaunch final_project_mixotics mixotics.launch` to run the `fixed_tag`, `vision`, `order_handler`, `update_scene` and `mover` nodes.
14. 


## Contents

This package contains:

- nodes:
    1. `fixed_tag`: generates a static transform between the robot and an AprilTag placed in front of the robot base
    2. `update_scene`: adds objects (i.e., ingredients, coasters) marked by AprilTags or color detection to the moveit planning scene
    3. `vision`: detects spaces occupied by cups in the custom drink holder
    4. `mover`: defines services for robot movements (i.e., grab, pour and set) using moveit planning and action libraries
    5. `order_handler`: processes orders from the user and generates a list of instructions used by the mover node
- config:
    1. `menu.xml`: configuration file for the menu including the recipe instructions for each drink
    2. `objects.yaml`: contains list of names for ingredients, coasters and tables (optional) as well as associated object dimensions
    3. `settings.yaml`: contains AprilTag code parameters such as tag family name and detection parameters
    4. `tags_color.yaml`: contains list of AprilTag IDs and sizes for each object (used with CV color detection)
    5. `tags.yaml`: contains list of AprilTag IDs and sizes for each object (used without CV color detection)
- launch: 
    1. `tag_finder.launch`: sets up the nodes for object detection and planning scene updates using a RealSense d435i camera
    2. `mixotics.launch`: master launch file for 
- srv:
    1. `ConfirmOrder.srv`: 
    2. `InventoryUpdate.srv`: 
    3. `Order.srv`:
    4. `PourInstr.srv`:
    5. `Refresh.srv`: checks planning scene updates from `update_scene/refresh`
    6. `SetInstr.srv`:
    7. `VerifyInstr.srv`:
- models:
    * STL files for ingredient bottle models 
- materials:
    * PDF files for AprilTags to be printed
- `package.xml`
    * Package configuration
- `CMakeLists.txt`
    * Build configuration
- `README.md`
    * Hello There!

## **Dependencies and Installation**
```
# run this command line tool while in a custom workspace
vcs import < mixotics.rosinstall
```


### *ROS Dependencies*
This package was developed and tested in ros-noetic. TODO

### *Python Dependencies*
All code for this package was developed and test in Python 3. TODO  

## **Execution**
```
ssh stuff
roslaunch panda_moveit_config panda_control_moveit_rviz.launch robot_ip:=robot.franka.de launch_franka_control:=false
roslaunch final_project_mixotics mixotics.launch
```
