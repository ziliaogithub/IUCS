#!/usr/bin/env python  

import os
import sys
import time
import copy
import string
import numpy as np
import scipy as sp

import roslib; roslib.load_manifest('scripting_util')



def angle_between(a, b):
  '''
    finds the angle between the given vectors in radians
  '''
  return np.arctan2(np.linalg.norm(np.cross(a,b)), np.dot(a,b));


