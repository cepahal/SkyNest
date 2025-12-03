# SkyNest
Project SkyNest Rapid Prototyping Simulator (RPS) is a way to quickly create and visualize robot ideas for group events, primarily FRC kickoff.

Using a low-poly CAD Library in Onshape is recommended, but not required. The dedicated library is based off of KrayonCAD and is linked below:

https://cad.onshape.com/documents/92c0a28558002254ede8c9dc/w/44650e4878bd1584808b443b/e/d20596289b639cd0447e3efd


****USAGE INSTRUCTIONS**** 

* Scale all models to inches, then export to STL. Only one color will be shown be used to save processing power and prevent crashes.

* Fields should be decimated as much as possible to not put strain on the Pybullet renderer. (<500k faces, preferably 100k)

* Label each joint with "dof_FRC_" then name it (e.g. dof_FRC_elevator) for the system to recognize the joint.

* The converter will NOT convert your file if any joints are not labeled! If it isn't a slider/revolute mate, it's best to convert it to a fastened mate to avoid errors.


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
