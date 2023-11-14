import cv2
import numpy as np

class HSVConfig:
    def __init__(self):
        self.h_min = [146]
        self.h_max = [179]
        self.s_min = [110]
        self.s_max = [255]
        self.v_min = [169]
        self.v_max = [255]
        self.hsv_Lower = (self.h_min[0], self.s_min[0], self.v_min[0])
        self.hsv_Upper = (self.h_max[0], self.s_max[0], self.v_max[0])


