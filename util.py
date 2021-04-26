import numpy as np 
import math
from scipy.signal import convolve2d

def get_bounds(config_space):
    #print('Made it to line 6, get_bounds')
    areas = []
    width_index = []
    
    a = 0
    for i in range(0, 400):
        #print('First for loop, get_bounds')
        same_area = False
        for j in range(0, 400):
            if(config_space[i][j] == 1):               
                a += 1
                width_index.append(j)
                
        if len(width_index) != 0:        
            for k in width_index:
                if(config_space[i][k] == 1):
                    same_area = True 
                 
        if(same_area == False and a != 0):
            areas.append(a)
            a = 0
             
    return areas 
    
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
    
