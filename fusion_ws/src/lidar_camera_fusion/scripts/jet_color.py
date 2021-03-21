# -*- coding: UTF-8 -*- 

import numpy as np
from matplotlib import cm

class Jet_Color(object):
    def __init__(self):
        colormap_int = np.zeros((256, 3), np.uint8)
        for i in range(0, 256, 1):
            colormap_int[i, 0] = np.int_(np.round(cm.jet(i)[0] * 255.0))
            colormap_int[i, 1] = np.int_(np.round(cm.jet(i)[1] * 255.0))
            colormap_int[i, 2] = np.int_(np.round(cm.jet(i)[2] * 255.0))
        
        self.jet = colormap_int
    
    def get_jet_color(self, val):
        idx = int(val)
        idx = max(idx, 0)
        idx = min(idx, 255)
        color = self.jet[idx]

        return [int(color[0]), int(color[1]), int(color[2])]
