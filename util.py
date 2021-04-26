import numpy as np 
import math
from scipy.signal import convolve2d

def get_bounds():
    raise NotImplementedError()
    
def get_config_space(map):
    kernel = np.ones((15, 15))
    config_space = convolve2d(map, kernel, mode='same')

    for i in range(0,400):
        for j in range(0,400):
            if(config_space[i][j] >= 400):
                config_space[i][j] = 1
                
    
    return config_space
    
def get_spawn_location():
    raise NotImplementedError()
    
