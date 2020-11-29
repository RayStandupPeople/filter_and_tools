from sys import argv
import numpy as np 
from matplotlib import pyplot as plt 
import time
import platform

import libs.types_pb2  # environment model data types


def read_pb_file():
    logfile_pb = libs.types_pb2.LogFile()
    with open("log/wholeFile_info_pb",'rb') as file:
        print(file)
        fileInfo_pb = file.read()
        logfile_pb.ParseFromString(fileInfo_pb)
    return logfile_pb

def init_window():
    win_handle = []
    grid = plt.GridSpec(3, 2, wspace=0.5, hspace=0.5)
    win_handle.append(plt.subplot(grid[0,0]))
    win_handle.append(plt.subplot(grid[1,0]))
    win_handle.append(plt.subplot(grid[2,0]))
    win_handle.append(plt.subplot(grid[0,1]))
    win_handle.append(plt.subplot(grid[1,1]))
    win_handle.append(plt.subplot(grid[2,1]))

    return win_handle

def plot_UserInfo(logfile_pb,win_hd):
    
    obj_left_flag =[]
    obj_mid_flag =[]
    obj_right_flag =[]
    obj_left_s_l =[]
    obj_mid_s_m =[]
    obj_right_s_r =[]

    for frame_ in logfile_pb.frame:
        obj_left_flag.append(frame_.userinfo.obj_left_flag)
        obj_mid_flag.append(frame_.userinfo.obj_mid_flag)
        obj_right_flag.append(frame_.userinfo.obj_right_flag)
        obj_left_s_l.append(frame_.userinfo.obj_left_s_l)
        obj_mid_s_m.append(frame_.userinfo.obj_mid_s_m)
        obj_right_s_r.append(frame_.userinfo.obj_right_s_r)
    t = np.linspace(0,logfile_pb.frame_num,logfile_pb.frame_num)

    # win_hd[0].plot(t,obj_mid_flag)
    # win_hd[0].set_title("obj_left_flag")
    win_hd[0].plot(t,obj_left_flag)
    win_hd[0].set_title("obj_mid_flag")
    # win_hd[2].plot(t,obj_right_flag)
    # win_hd[2].set_title("obj_right_flag")
    # win_hd[3].plot(t,obj_left_s_l)
    # win_hd[3].set_title("obj_left_s")
    win_hd[4].plot(t,obj_mid_s_m)
    win_hd[4].set_title("obj_mid_s")
    # win_hd[5].plot(t,obj_right_s_r)
    # win_hd[5].set_title("obj_right_r")

def plot_loclization(logfile_pb,win_hd):
    latRest_X = []
    lonRest_Y = []
    t = np.linspace(0,logfile_pb.frame_num,logfile_pb.frame_num)
    for frame_ in logfile_pb.frame:
        latRest_X.append(frame_.localization.latRest_X)
        lonRest_Y.append(frame_.localization.lonRest_Y)
    win_hd[3].plot(latRest_X,lonRest_Y,'o')
    win_hd[3].set_title("Location")
    win_hd[1].plot(t,latRest_X)
    win_hd[1].set_title("Location_X")
    win_hd[2].plot(t,lonRest_Y)
    win_hd[2].set_title("Location_Y")
    
if __name__ == "__main__":
    pb_file = read_pb_file()
    win_hd  = init_window()
    plot_UserInfo(pb_file,win_hd)
    plot_loclization(pb_file,win_hd)
    plt.show()