# Final_Project_Mixotics Package: Sir Mix-a-lot, the cafe bot!

Authors and Maintainers: 
- **Ryan King-Shepard (RKS)**
- **Marshall Johnson**
- **Matt Short**
- **Eric Codrea**
- **Pranav Kaarthik**  


## **Description**
This package enables the Franka robot to create and serve a variety of drinks.


## **User Guide**

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
14. The robot can now fulfill an order! Run `rosservice call /process_order {"orders= 'water', 'water', 'lemonade'"}` to have Sir Mix-a-lot make a drink! The acceptable drink commands are "water", "lemonade", "iced tea"


## High Level Concepts and Overall System Architecture

The process loop of the robot is as follows:
1. Take an order for the customer and prepare to execute the order.
2. Survey the workspace and update the planning scene with the location of each drink
3. Execute the grab service - the grab service involves the robot resetting to a default location where it has good access to the entire workspace to be used, a cartesian motion to bring the robotic arm to the first ingredient with the grippers in place to grasp the bottle, and finally a grasping action that allows for the bottle to be lightly grasped without squeezing too hard.
4. Execute the pour service - the pour service involves the robot executing a cartesian motion to bring the ingredient to a spot offset from the coasters location where it can be squeezed and pour an ingredient into each cup, a joint angle command to tilt the end effector with the bottle which allows the pour to be directly into the cup, a squeeze action which uses force control to squeeze the grippers and finally a reverse of the joint angle command previously sent to bring the gripper horizontal with the table.
5. Execute the stock service - the stock service involves the robot executing a cartesian trajectory to return the robotic arm to the initial drink location, and a gripper position command to open the grippers gently and deposit the ingredient back at its starting location.
6. Repeat 3-5 in order to fulfil each drink that the customer requires. Multi-ingredient drinks require these steps to be executed multiple times for each ingredient.

The order handler further takes into account whether or not the robot successfully completed each of the prior steps and if an error is received, stops the robot.


## Contents

This package contains:

- nodes:
    1. `fixed_tag`: generates a static transform between the robot and an AprilTag placed in front of the robot base
    2. `update_scene`: adds objects (i.e., ingredients, coasters) marked by AprilTags or color detection to the moveit planning scene
    3. `vision`: detects spaces occupied by cups in the custom drink holder
    4. `mover`: defines services for robot movements (i.e., grab, pour and set) using moveit planning and action libraries
    5. `order_handler`: processes orders from the user and generates a list of instructions used by the mover node
    6. `voice`: processes verbal orders from the user and uses service proxy to integrate with order_handler. Integration to move robot using verbal orders is still in development. Verbal order processing alone is functional.
- config:
    1. `menu.xml`: configuration file for the menu including the recipe instructions for each drink
    2. `objects.yaml`: contains list of names for ingredients, coasters and tables (optional) as well as associated object dimensions
    3. `settings.yaml`: contains AprilTag code parameters such as tag family name and detection parameters
    4. `tags_color.yaml`: contains list of AprilTag IDs and sizes for each object (used with CV color detection)
    5. `tags.yaml`: contains list of AprilTag IDs and sizes for each object (used without CV color detection)
- launch: 
    1. `tag_finder.launch`: sets up the nodes for object detection and planning scene updates using a RealSense d435i camera
    2. `mixotics.launch`: master launch file for the robot - this runs all the nodes required to update the planning scene as well as move the robot 
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


## *ROS Dependencies and Libraries Used*
This package was developed and tested in ros-noetic. 

1. *pyrealsesnse2* - Library to use and run the Intel Realsense D435i depth camera.
2. *MoveIt* - The motion planning library used to provide the trajectories that enabled the robot to perform its tasks.
3. *libfranka/ franka_ros* - This library allowed  low-level control of the Emika Franka Panda robot and control of the gripper. 


### *Python Dependencies*
All code for this package was developed and test in Python 3.


## *Physical Equipment*
1. Emika Franka Panda robot
2. Intel Realsense D435i depth sensing camera
3. [4 oz. washbottles](https://www.grainger.com/product/LAB-SAFETY-SUPPLY-Wash-Bottle-4-oz-Labware-Capacity-6FAV7) (3x)
4. [9 oz. solo cups](https://www.amazon.com/Disposable-Party-Plastic-Cups-Pack/dp/B092FY8VSJ/ref=sr_1_22?keywords=solo%2Bcups&qid=1638291607&sr=8-22&th=1) (4x)
5. Lasercut acrylic cup flight
6. 3D printed Apriltag Coasters (3x for washbottles and 4x for flight)
7. Safety Spillage Bins
8. Tables

