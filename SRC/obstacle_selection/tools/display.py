#!usr/bin python
import numpy as np
from matplotlib import pyplot as plt
import sys
sys.path.append("..")
import proto.obstacleSel_pb2


class Plot:
    def __init__(self):
        self.fig = plt.figure()
        self.grid = plt.GridSpec(5, 2, wspace=0.5, hspace=0.5) # window set
        self.ax1 = self.fig.add_subplot(self.grid[0,0:2])
        self.ax2 = self.fig.add_subplot(self.grid[1:6,0])
        self.ax3 = self.fig.add_subplot(self.grid[1:6,1])
        self.ax1.axis([-150,100,-200,-100])
        self.ax2.axis([-20,20,-40,100])
        self.ax3.axis([-20,20,-40,200])

        self.line1,    = self.ax1.plot(0,0)
        self.line2_1,  = self.ax2.plot(0,0)
        self.line2_2,  = self.ax2.plot(0,0,'o')
        self.line3_1,  = self.ax3.plot(0,0)
        self.line3_2,  = self.ax3.plot(0,0,'o')


    def plot_flat_path(self, path):
        flat_x_list =[]
        flat_y_list =[]
        for path_node_ in path.path_node:
            flat_x_list.append(path_node_.flat_x)
            flat_y_list.append(path_node_.flat_y)
        self.line1.set_xdata(flat_x_list)
        self.line1.set_ydata(flat_y_list)

    def plot_vehicle_path(self, path):
        car_x_list =[]
        car_y_list =[]
        for path_node_ in path.path_node:
            car_x_list.append(path_node_.car_x)
            car_y_list.append(path_node_.car_y)
        self.line2_1.set_xdata(car_x_list)
        self.line2_1.set_ydata(car_y_list)

    def plot_vehicle_obstalce(self, obstalce_list):
        car_x_list =[]
        car_y_list =[]
        for obstalce_ in obstalce_list.obstacle:
            car_x_list.append(obstalce_.pos_x)
            car_y_list.append(obstalce_.pos_y* -1)
        self.line2_2.set_xdata(car_y_list)
        self.line2_2.set_ydata(car_x_list)

    def plot_frenet_path(self, path):
        s_list =[]
        d_list =[]
        for path_node_ in path.path_node:
            s_list.append(path_node_.s)
            d_list.append(path_node_.d)
        self.line3_1.set_xdata(d_list)
        self.line3_1.set_ydata(s_list)

    def plot_frenet_obstalce(self, obstalce_list):
        s_list =[]
        d_list =[]
        for obstalce_ in obstalce_list.obstacle:
            s_list.append(obstalce_.pos_s)
            d_list.append(obstalce_.pos_d * -1)
        self.line3_2.set_xdata(d_list)
        self.line3_2.set_ydata(s_list)

    
def read_global_path_file(file_name):
    global_x=[]
    global_y=[]
    with open(file_name) as in_file:
        lines = in_file.readlines()
        for line in lines:
            line_ = line.strip().split()
            global_x.append(line_[0])
            global_y.append(line_[1])
    plt.plot(global_x,global_y)
    plt.plot(global_x[0], global_y[0], 'r*')
    plt.plot(global_x[-1],global_y[-1],'y*')

def read_protobuf_file(file_name):
    pb_file = proto.obstacleSel_pb2.LogFile()
    with open(file_name) as file:
        pb_str = file.read()
        pb_file.ParseFromString(pb_str) 
    return pb_file 


if __name__ == "__main__":
    file_pb = read_protobuf_file("../data/data_convert") 
    p = Plot()
    p.plot_vehicle_path(file_pb.path)
    p.plot_flat_path(file_pb.path)
    p.plot_vehicle_obstalce(file_pb.obstacle_list)
    p.plot_frenet_obstalce(file_pb.obstacle_list)
    p.plot_frenet_path(file_pb.path)
    plt.show()







        
