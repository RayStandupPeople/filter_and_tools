#!usr/bin python
import numpy as np
from matplotlib import pyplot as plt
import sys
sys.path.append("..")
import proto.obstacleSel_pb2


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
    plt.plot(global_x[0],global_y[0],'r*')
    plt.plot(global_x[-1],global_y[-1],'y*')

def read_protobuf_file(file_name):
    pb_file = proto.obstacleSel_pb2.LogFile()
    with open(file_name) as file:
        pb_str = file.read()
        pb_file.ParseFromString(pb_str) 
    return pb_file 
def plot_vehicle_path(path):
    car_x_list =[]
    car_y_list =[]
    for path_node_ in path.path_node:
        car_x_list.append(path_node_.car_x)
        car_y_list.append(path_node_.car_y)
    plt.plot(car_x_list,car_y_list)

def plot_flat_path(path):
    flat_x_list =[]
    flat_y_list =[]
    for path_node_ in path.path_node:
        flat_x_list.append(path_node_.flat_x)
        flat_y_list.append(path_node_.flat_y)
    plt.plot(flat_x_list,flat_y_list)

def plot_frenet_path(path):
    s_list =[]
    d_list =[]
    for path_node_ in path.path_node:
        s_list.append(path_node_.s)
        d_list.append(path_node_.d)
    plt.plot(d_list,s_list)

def plot_vehicle_obstalce(obstalce_list):
    car_x_list =[]
    car_y_list =[]
    for obstalce_ in obstalce_list.obstacle:
        car_x_list.append(obstalce_.pos_x)
        car_y_list.append(obstalce_.pos_y* -1)
    plt.plot(car_y_list,car_x_list,'o')

def plot_frenet_obstalce(obstalce_list):
    s_list =[]
    d_list =[]
    for obstalce_ in obstalce_list.obstacle:
        s_list.append(obstalce_.pos_s)
        d_list.append(obstalce_.pos_d)
    plt.plot(d_list,s_list,'o')


if __name__ == "__main__":
    file_pb = read_protobuf_file("../data/data_convert")
    # read_global_path_file("../../../log/geely.csv")
    plot_vehicle_path(file_pb.path)
    plot_flat_path(file_pb.path)
    plot_vehicle_obstalce(file_pb.obstacle_list)
    plot_frenet_obstalce(file_pb.obstacle_list)
    plot_frenet_path(file_pb.path)
    plt.show()







        
