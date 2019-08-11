#!/usr/bin/env python
import numpy as np
from bresenham import bresenham

class Grid:
    def __init__(self):
        self.x_lim = []
        self.y_lim = []
        self.res = []
        self.matrix = []
        self.empty_idxs = []
        self.wall_idxs = []
