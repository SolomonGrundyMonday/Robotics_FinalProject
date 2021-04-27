# Robotics_FinalProject

This project has a couple of special requirements to run correctly.  First, you will need to place the python file "util.py" into the same directory as the
"Mapper.py" controller code.  The util.py file defines a bunch of utility functions that are needed to compute the inverse kinematics of the manipulator arm and
the computations on the map/configuration space files.  We decided to place this code in its own file in order to make our project code as clean and easy to read
as possible.  You may also need to use the Webots wizard to create a new controller and copy the "Mapper.py" controller code into the newly created controller file.
To run the robot, perform the following steps:
  1) Open the "final.wbt" world file in Webots.
  2) Generate a new controller with the create controller wizard in webots.
  3) (Only do this step if Webots throws any controller errors, or the controller cannot be selected in the controller selection window in Webots.)
     Replace the generic controller code with the code from the file in this repo "Mapper.py".
  4) Place the file "util.py" into the exact same directory that the new controller file is stored in.
  5) Run the simulation in Webots.
  6) NOTE:  The full sequence takes a few hours to complete, so if you do not have time to complete it, make sure to download the "map.npy" file from this repo
     and change the value of "mode" variable to 'repair' before you run the code.  This will allow you to watch repairs being made to the damaged floor sections
     without waiting for the mapping sequence to complete (the mapping sequence takes a few hours to complete due to the size of the arena and speed of the robot).
     The "map.npy" file contains the output of a mapping sequence that we ran on the "final.wbt" map, thus watching the mapping routine is only necessary if you are 
     interested or have the time to spare.
