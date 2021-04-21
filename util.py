import numpy as np
import math

# This gets the coordinates for the damaged areas on the config space map.
def get_bounds():
    raise NotImplementedError()
   
# This takes the output from the autonomous mapper and converts it to config space map.
def get_config_space():
    raise NotImplementedError()

# This takes the list of boundaries for damaged areas and decides where to spawn the manipulator arm and repair materials
def get_spawn_location():
    raise NotImplementedError()
    
