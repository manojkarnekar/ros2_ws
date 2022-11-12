import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

try:
    df = pd.read_csv("jetson_xivir_log.csv")
    t = ['CPU1', 'CPU2','CPU3','CPU4','CPU5','CPU6','GPU', 'RAM','EMC', 'IRAM', 'Temp AO', 'Temp CPU', 'Temp GPU', 'Temp PLL', 'Temp thermal', 'power cur', 'power avg','fan']
    for it in t:
        try:
            ax = df.plot(x="time", y=it, style='b')
            ax.set_ylabel(it)
        except:
            pass
    plt.show()

    fig, axes = plt.subplots(nrows=6, ncols=3)
    k = 0
    fig = plt.figure()
    spacing = 0.100
    for i in range(6):
        for j in range(3):
            # print(i,j)
            try:
                df[t[k]].plot(ax=axes[i,j], style='b'); axes[i,j].set_title(t[k])
                k+=1
            except:
                k+=1
    # set the spacing between subplots
    plt.subplots_adjust(left=0.1,
                        bottom=0.1, 
                        right=0.9, 
                        top=0.9, 
                        wspace=0.9, 
                        hspace=0.9)
    plt.show()
    
except KeyboardInterrupt:
    print("Closed with CTRL-C")

