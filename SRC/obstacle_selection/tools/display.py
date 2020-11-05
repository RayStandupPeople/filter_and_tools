#!usr/bin python
import numpy as np
from matplotlib import pyplot as plt

data=[]
x=[]
y=[]
s=[]
dx=[]
with open("../../../log/highway_map.csv",'r') as in_file:
    lines = in_file.readlines()
for line in lines:
    line_ = line.strip().split()
    x.append(line_[0])
    y.append(line_[1])
    s.append(line_[2])

plt.plot(x,y,'o')
plt.plot(x[0],y[0],'r*')
plt.plot(x[-1],y[-1],'y*')
while True:
    plt.show()

        
