import numpy as np
from matplotlib import pyplot as plt 
import math
from scipy.signal import convolve2d

def get_bounds():
    raise NotImplementedError()
    
def get_config_space():
    map = np.load('map.npy')
    kernel = np.ones((3, 3))
    config_space = convolve2d(map, kernel, mode='same')

    for i in range(0,400):
        for j in range(0,400):
            if(config_space[i][j] >= 600):
                config_space[i][j] = 1
                
    plt.imshow(config_space, cmap='gray')
    plt.show()
    return config_space
    
def get_spawn_location():
    raise NotImplementedError()
    
get_config_space()