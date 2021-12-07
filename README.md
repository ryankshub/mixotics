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
3. Install any python dependencies needed to run the software. Refer to the [python dependencies](#python-dependencies) section.
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


## Contents

This package contains:

- nodes:
    1. `fixed_tag`: Generates a static transform between the robot and an 
            AprilTag placed in front of the robot base.
    2. `update_scene`: Adds objects (i.e., ingredients, coasters) marked by 
            AprilTags or color detection to the moveit planning scene.
    3. `vision`: Detects occupied spaces in the custom drink holder.
    4. `mover`: Defines services for robot movements (i.e., grab, pour, and stock) 
            using moveit planning and action libraries
    5. `order_handler`: Processes user's orders and generates a list of 
            instructions used by the mover node. Once all instructions for an order 
            are complete, sends confirmation to the user.
    6. `voice`: Processes verbal orders from the user and uses service proxy to
            integrate with `order_handler`. Note: Integration to move robot using 
            verbal ordering is still in development. Verbal order processing alone is 
            functional.
- config:
    1. `menu.xml`: configuration file for the menu containing the name, ingredients, recipe instructions for each drink
    2. `objects.yaml`: contains list of names for ingredients, coasters and tables (optional) as well as associated object dimensions
    3. `settings.yaml`: contains AprilTag code parameters such as tag family name and detection parameters
    4. `tags_color.yaml`: contains list of AprilTag IDs and sizes for each object (used with CV color detection)
    5. `tags.yaml`: contains list of AprilTag IDs and sizes for each object (used without CV color detection)
- launch: 
    1. `tag_finder.launch`: sets up the nodes for object detection and planning scene updates using a RealSense d435i camera
    2. `mixotics.launch`: master launch file for the robot - this runs all the nodes required to update the planning scene as well as move the robot 
- srv:
    1. `ConfirmOrder.srv`: Request/Response for indicting an order is complete. Includes value to determine if order was mase successfully or not
    2. `InventoryUpdate.srv`: Request for updating the whether an ingredient is available of not
    3. `Order.srv`: Request contain a list of drinks the user wants
    4. `PourInstr.srv`: Request structure used to tell the robot to pour an ingredient in a certain cup
    5. `Refresh.srv`: checks planning scene updates from `update_scene/refresh`
    6. `SetInstr.srv`: Request used to tell the robot which ingredient to grab or stock
    7. `VerifyInstr.srv`: Request used to confirm robot has completed instuction from `order_handler`
- msg:
    1. `CupDetection.msg`: Array to indicate which coaster spot are filled with cups.
- src:
    1. `menu_utilities`: custom python package which is responsible for codifing menu and providing helper functions for processing/tracking orders
- test:
    Location of unit and integration testing
- models:
    * STL files for ingredient bottle models 
- materials:
    * PDF files for AprilTags to be printed
- bag: 
    * location for Rosbag file used for testing
- `package.xml`
    * Package configuration
- `CMakeLists.txt`
    * Build configuration
- `README.md`
    * Hello There!


## High Level Concepts and Overall System Architecture

The process loop of the robot is as follows:
1. Take an order from the customer and validate it. Validation ensures the drink is on the menu and the ingredients are in stock to make the drink. 
2. If the order is valid, the customer is given a postive order number; the system will use this number to track it. If the order is not valid, the system returns an order number of 0
and an explanation of why the order could not be accepted.
3. Once the order is accepted, the recipes of each drink in the order are processed into a list of instructions for the robot to execute. Instructions either tell the robot to grab an ingredient, pour an amount into a cup, or stock the ingredient. Until the order is complete, each instruction is sent to the robot one at a time.
4. Meanwhile, the robot surveys the workspace and update the planning scene with the location of ingredients and coasters. Based on the type of the instruction, the robot can perform one of the following in 5-7:
5. Execute the grab service - the grab service involves the robot resetting to a default location where it has good access to the entire workspace to be used, a cartesian motion to bring the robotic arm to the first ingredient with the grippers in place to grasp the bottle, and finally a grasping action that allows for the bottle to be lightly grasped without squeezing too hard.
6. Execute the pour service - the pour service involves the robot executing a cartesian motion to bring the ingredient to a spot offset from the coasters location where it can be squeezed and pour an ingredient into each cup, a joint angle command to tilt the end effector with the bottle which allows the pour to be directly into the cup, a squeeze action which uses force control to squeeze the grippers and finally a reverse of the joint angle command previously sent to bring the gripper horizontal with the table.
7. Execute the stock service - the stock service involves the robot executing a cartesian trajectory to return the robotic arm to the initial drink location, and a gripper position command to open the grippers gently and deposit the ingredient back at its starting location.
8. Repeat 4-7 in order to fulfil each drink that the customer requires. Multi-ingredient drinks require these steps to be executed multiple times for each ingredient.

## Demonstrations

### Serving a Drink
![Drink Serving Demo](videos/demo_final.gif)

### Verbal Order Processing
This is a link to our verbal order processing system in action: https://www.youtube.com/watch?v=76QoueeHdhA&t=9s

## *ROS Dependencies and Libraries Used*
This package was developed and tested in ros-noetic. 

1. *pyrealsesnse2* - Library to use and run the Intel Realsense D435i depth camera.
2. *MoveIt* - The motion planning library used to provide the trajectories that enabled the robot to perform its tasks.
3. *libfranka/ franka_ros* - This library allowed  low-level control of the Emika Franka Panda robot and control of the gripper. 



## *Python Dependencies*
All code for this package was developed and test in Python 3. 

Additionally, users will need to install [OpenCv](https://docs.opencv.org/4.5.4/) using:
```python
pip3 install opencv-python
```


## *Physical Equipment*
1. Emika Franka Panda robot
2. Intel Realsense D435i depth sensing camera
3. [4 oz. washbottles](https://www.grainger.com/product/LAB-SAFETY-SUPPLY-Wash-Bottle-4-oz-Labware-Capacity-6FAV7) (3x)
4. [9 oz. solo cups](https://www.amazon.com/Disposable-Party-Plastic-Cups-Pack/dp/B092FY8VSJ/ref=sr_1_22?keywords=solo%2Bcups&qid=1638291607&sr=8-22&th=1) (4x)
5. Lasercut acrylic cup flight
6. 3D printed Apriltag Coasters (3x for washbottles and 4x for flight)
7. Safety Spillage Bins
8. Tables

