# SkyNest
Project SkyNest Rapid Prototyping Simulator (RPS) is a way to quickly create and visualize robot ideas for group events, primarily FRC kickoff.


****USAGE INSTRUCTIONS**** 

* Scale all models to inches, then export to STL. Only one color will be shown be used to save processing power and prevent crashes.

* Fields should be decimated as much as possible to not put strain on the Pybullet renderer. (<500k faces, preferably 100k)

* Label each joint with "dof_FRC_" then the name for the system to recognize the joint.


Available Actions - [move,rotate,lift,intake,outtake]

Available Targets - [base,jointName,gamepiece,object]


* "base" represents the robot. It is controlled with "move" and "rotate" only.

* "gamepiece" and "object" can be used interchangeably, and must be configured beforehand.


****Dependencies****

PyQt5

Pybullet

onshape-to-robot


Running project - 

exe -> Skynest_RPS_4328 -> dist -> Skynest_RPS_4328 -> Skynest_RPS_4328.exe


* If you want to run the python file, you must change the functions around load_urdf_near_path() to .py instead of .exe to properly open the converter.

* The converter can also be run separately. 


# Please let me know if you have any suggestions/bugs and I'll try to fix them ASAP!
