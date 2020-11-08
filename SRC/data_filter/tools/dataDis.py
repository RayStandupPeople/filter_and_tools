#!usr/bin/python
#coding=utf-8
import numpy as np
from matplotlib import pyplot as plt


with open("../data/data_convert") as f:
    line = f.readline().strip()
    linestr = line.split(" ")
    linenum_1 = map(float,linestr)
    line = f.readline().strip()
    linestr = line.split(" ")
    linenum_2 = map(float,linestr)
    line = f.readline().strip()
    linestr = line.split(" ")
    linenum_3 = map(float,linestr)
    line = f.readline().strip()
    linestr = line.split(" ")
    linenum_4 = map(float,linestr)



# data1 = [35.2, 34.8, 35.1, 34.2, 33.4, 32.1, 52.3, 50.2, 30.4, 29.4, 28.7, 20.1, 26.5, 25.4, 23.9]
# data2 = [35.2, 34.8, 35.1, 34.2, 34.8, 34.2, 34.2, 34.2, 33.4, 32.1, 30.4, 29.4, 28.7, 26.5, 25.4]
data1 = linenum_1
data2 = linenum_2
data3 = linenum_3
data4 = linenum_4
x = np.linspace(1,len(data1),len(data1))

data12 = data1 + data2  # for x axis limit
data34 = data3 + data4

p1=plt.subplot(2,2,1)
p1.plot(x,data1)
p1.axis([0,len(data1),min(data12),max(data12)*1.1])
# print(max(data12))
plt.xlabel("original data1")
p2=plt.subplot(2,2,2) 
p2.plot(x,data2)
p2.axis([0,len(data1),min(data12),max(data12)*1.1])
plt.xlabel("filted data1")


p3=plt.subplot(2,2,3)
p3.plot(x,data3)
p3.axis([0,len(data3),min(data34),max(data34)*2])
plt.xlabel("original data2")
p4=plt.subplot(2,2,4) 
p4.plot(x,data4)
p4.axis([0,len(data3),min(data34),max(data34)*2])
plt.xlabel("filted data2")


plt.show()


