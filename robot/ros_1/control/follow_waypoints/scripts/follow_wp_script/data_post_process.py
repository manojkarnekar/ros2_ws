import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import math

df = pd.read_csv("pose_data.csv")

pose_data = open("pp_pose_data.csv", mode='w+')
pose_data.write("{},{},{},{}\n".format("P_x","P_y", "O_z", "O_w"))

old_x = 0
old_y = 0

for P_x, P_y, O_z, O_w in zip(df["P_x"], df["P_y"], df["O_z"], df["O_w"] ):
    curr_dis = abs(math.sqrt((P_x - old_x)**2 + (P_y - old_y)**2))
    
    if curr_dis > 1.0:    
        print(P_x, P_y)
        
        pose_data.write("{},{},{},{}\n".format(P_x, P_y, O_z, O_w))
        
        old_x = P_x
        old_y = P_y


pose_data.close()


