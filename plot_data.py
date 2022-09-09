

from itertools import count
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


plt.style.use('bmh')

x_vals = []
y_vals = []

index = count()


def animate(i):
    data = pd.read_csv('/home/panda/ws_moveit/src/effort_data.csv')
    #x = data['x_value']
   
    y1 = data['joint_effort[0]']
    y2 = data['joint_effort[1]']
    y3 = data['joint_effort[2]']
    y4 = data['joint_effort[3]']
    y5 = data['joint_effort[4]']
    y6 = data['joint_effort[5]']
    y7 = data['joint_effort[6]']
    y8 = data['joint_effort[7]']
    y9 = data['joint_effort[8]']

    plt.cla()

    plt.plot(  y1, label='joint_effort[0]')
    plt.plot(  y2, label='joint_effort[1]') 
    plt.plot(  y3, label='joint_effort[2]')
    plt.plot(  y4, label='joint_effort[3]')
    plt.plot(  y5, label='joint_effort[4]')
    plt.plot(  y6, label='joint_effort[5]')
    #plt.plot(  y7, label='joint_effort[6]')
    #plt.plot(  y8, label='joint_effort[7]') 
    #plt.plot(  y9, label='joint_effort[8]')
    
    plt.legend(loc='upper left')
    plt.tight_layout()


ani = FuncAnimation(plt.gcf(), animate, interval=1)

plt.tight_layout()
plt.show()
    
   
    


   
   

   