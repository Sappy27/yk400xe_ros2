# yk400xe_ros2
A package to use Yamaha YK400XE robot with ROS2

# How to use
Clone this repo in your src folder.  
Plug the Ethernet cable to the RCX340 controller and to you computer.  
Using the PBX, initialize the Ethernet connection of the controller.  
Setup you LAN interface ip to be static and 192.168.0.XXX (XXX belongs to [3,255]) or adapt it depending on you controller ip.

To start the communication with the controller :   
``run ros2 run rcx340_bringup controller_communication``  
The default IP and port of the controller should be : 192.168.0.2:23  
If not, they should be remaped adding this to the previous command :   
``--ros_args -p controller_ip:="XXX.XXX.XXX.XXX" -p controller_port:=XX``  
TODO --> make a .yaml
  
You should see a message saying "Welcome to RCX340" displayed in the terminal. 
Then :  
``run ros2 launch yk400xe_description yk400xe_description.launch``   
to load the URDF and start the joint state publisher.  
You can then start rviz with ``rviz2`` to visualize the robot.  
TODO --> make a rviz folder and start the rviz with launch
  
You can now control the robot using either the Graphic User Interfaces  
``ros2 run yk400xe_control GUI_command_joints.py`` for joint values control 
or  
``ros2 run yk400xe_control GUI_command_xy.py`` for end effector coordinates control  
or use directly the available services ``/command/moveP_joints`` or ``/command/moveP_xy`` in your code (see controller_communication.cpp)
