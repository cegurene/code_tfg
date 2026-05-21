<em>  GRAPHICAL USER INTERFACE </em>


![Badge en Desarollo](https://img.shields.io/badge/STATUS-OK-green)  
![Badge ROS2](https://img.shields.io/badge/ROS2-OK-green) 

# Initial description
This repository has the gui to control the New Leg Replica. Here you can move the Leg with sliders, puntual setpoints, senoidal trajectories or setpoints extracted from external csv archives.

# New Leg Replica GUI

The archive where the design is obtained is 'GUI_2.ui'. To open and modify it you need to install [pyQT6](https://pypi.org/project/PyQt6/)
The python node to launch is 'ros_backend_peakCAN.py'. This one takes the before design and provides the back-end logic.

# Controller PID desginer GUI

The archive where the design is obtained is 'GUI_PID.ui'. To open and modify it you need to install [pyQT6](https://pypi.org/project/PyQt6/)
The python node to launch is 'ros_backend_PID.py'. This one takes the before design and provides the back-end logic.


