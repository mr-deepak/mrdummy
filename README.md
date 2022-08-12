# mrdummy
5 DOF robotic arm with future plans

To start Moveit simulation
"""
roslaunch mrdummy_movieit_gazebo demo.launch
"""

To start Gazebo simulation
"""
roslaunch description mrdummy.launch
"""

To start both gazebo and moveit together
"""
roslaunch mrdummy_movieit_gazebo demo_gazebo.launch
"""

Moving robot with program 
The cpp file will be found inside the package programs. Open in new terminal and run 
"""
rosrun programs cp.cpp
"""
