# Final_Project_Mixotics Package: Sir Mix-a-lot, the cafe bot!

Authors and Maintainers: 
- **Ryan King-Shepard (RKS)**
- **Marshall Johnson**
- **Matt Short**
- **Eric Codrea**
- **Pranav Kaarthik**  

## **Description**
This package enables the Franka robot to create and serve a variety of drinks.

## Contents

This package contains:

- nodes:
    1. `fixed_tag`: generates a static transform between the robot and an AprilTag placed in front of the robot base
    2. `update_scene`: adds objects (i.e., ingredients, coasters) marked by AprilTags or color detection to the moveit planning scene
    3. `vision`: detects spaces occupied by cups in the custom drink holder
    4. `mover`: 
- config:
    1. `menu.xml`: configuration file for the menu including the recipe instructions for each drink
    2. `objects.yaml`: contains list of names for ingredients, coasters and tables (optional) as well as associated object dimensions
    3. `settings.yaml`: contains AprilTag code parameters such as tag family name and detection parameters
    4. 'tags_color.yaml': contains list of AprilTag IDs and sizes for each object (used with CV color detection)
    5. 'tags_color.yaml': contains list of AprilTag IDs and sizes for each object (used without CV color detection)
- launch: 
    1. `tag_finder.launch`: sets up the nodes for object detection and planning scene updates using a RealSense d435i camera
    2. 
- srv:
    1. grab
    2. InventoryUpdate
    3. Order
    4. Refresh
    5. PourInstr
    6. TO DO
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
Add info for mixotics.rosinstall installation 

### *ROS Dependencies*
This package was developed and tested in ros-noetic. TODO

### *Python Dependencies*
All code for this package was developed and test in Python 3. TODO  

## **Execution**

TODO
```
roslaunch final_project_mixotics tag_finder.launch demo:=false color:=true
```
