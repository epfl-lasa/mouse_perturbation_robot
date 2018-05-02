# -*- coding: utf-8 -*-
"""
Script to read in the KUKA log files for the Hasler project experiments
"""
import numpy as np

data = []

with open('Run1_Iason.txt') as f:
    next(f)
    data = [[x for x in line.split()] for line in f]

data = np.array(data).astype(np.float)

start = np.where(data[:,5])[0][0]
time = data[start:,0]
ee_pose = data[start:,1:4]
sf = data[start:,7]
rho = data[start:,8]
event = data[start:,-1]

event_indices = np.where(event==15)[0]
#event_indices = 1+np.where(event[:-1]!=event[1:])[0]
#ends_of_trials = event_indices[np.where(event[event_indices]==0)[0]]
#error_count = event_indices[np.where(event[event_indices]==15)[0]]
