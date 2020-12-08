#!usr/bin python
import numpy as np
import copy
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
        self.ax2.axis([-20,20,-30,60])
        self.ax3.axis([-20,20,-30,120])

        self.line1,    = self.ax1.plot(0,0)
        self.line2_1,  = self.ax2.plot(0,0,'--y')         # ref line
        self.line2_2,  = self.ax2.plot(0,0,'o')     # obstacle
        self.line2_3,  = self.ax2.plot(100,0,'or')  # selected obj
        self.line2_4,  = self.ax2.plot(0,0,'b')         # left line
        self.line2_5,  = self.ax2.plot(0,0,'b')         # right line

        self.line3_1,  = self.ax3.plot(0,0,'--y')
        self.line3_2,  = self.ax3.plot(0,0,'o')     #frenet obstacle
        self.line3_3,  = self.ax3.plot(0,0,'b')     #frenet leftline
        self.line3_4,  = self.ax3.plot(0,0,'b')     #frenet righttline
        self.line3_5,  = self.ax3.plot(100,0,'or')     #frenet cipv



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
            # print("car_x: %f, car_y: %f"%(path_node_.car_x,path_node_.car_y))
        self.line2_1.set_xdata(car_x_list)
        self.line2_1.set_ydata(car_y_list)

    def plot_vehicle_lane(self, l_line,r_line):
        car_x_list_l =[]
        car_y_list_l =[]
        car_x_list_r =[]
        car_y_list_r =[]
        for path_node_ in l_line.path_node:
            car_x_list_l.append(path_node_.car_x)
            car_y_list_l.append(path_node_.car_y)
            # print("car_x: %f, car_y: %f"%(path_node_.car_x,path_node_.car_y))
        for path_node_ in r_line.path_node:
            car_x_list_r.append(path_node_.car_x)
            car_y_list_r.append(path_node_.car_y)
        self.line2_4.set_xdata(car_x_list_l)
        self.line2_4.set_ydata(car_y_list_l)
        self.line2_5.set_xdata(car_x_list_r)
        self.line2_5.set_ydata(car_y_list_r)

    def plot_vehicle_obstalce(self, obstalce_list, cipv_obj):
        car_x_list =[]
        car_y_list =[]
        for obstalce_ in obstalce_list.obstacle:
            car_x_list.append(obstalce_.pos_x)
            car_y_list.append(obstalce_.pos_y* -1)
        self.line2_2.set_xdata(car_y_list)
        self.line2_2.set_ydata(car_x_list)
        if cipv_obj.cipv_obstacle.pos_x!=0 and cipv_obj.cipv_obstacle.pos_y !=0:
            self.line2_3.set_xdata(cipv_obj.cipv_obstacle.pos_y * -1)
            self.line2_3.set_ydata(cipv_obj.cipv_obstacle.pos_x)

    def plot_frenet_path(self, path):
        s_list =[]
        d_list =[]
        for path_node_ in path.path_node:
            s_list.append(path_node_.s)
            d_list.append(path_node_.d)
        self.line3_1.set_xdata(d_list)
        self.line3_1.set_ydata(s_list)
    
    def plot_frenet_lane(self, path, lanewidth,cipv_obj):
        s_list =[]
        d_list_left =[]
        d_list_right =[]
        for path_node_ in path.path_node:
            s_list.append(path_node_.s)
            d_list_left.append(path_node_.d - lanewidth/2)
            d_list_right.append(path_node_.d + lanewidth/2)

        self.line3_3.set_xdata(d_list_left)
        self.line3_3.set_ydata(s_list)
        self.line3_4.set_xdata(d_list_right)
        self.line3_4.set_ydata(s_list)
        print(cipv_obj.cipv_obstacle.pos_d)
        print(cipv_obj.cipv_obstacle.pos_s)
        print(cipv_obj.cipv_obstacle.pos_x)
        print(cipv_obj.cipv_obstacle.pos_y)

        if cipv_obj.cipv_obstacle.pos_s!=0 and cipv_obj.cipv_obstacle.pos_d !=0:
            self.line3_5.set_xdata(cipv_obj.cipv_obstacle.pos_d * -1)
            self.line3_5.set_ydata(cipv_obj.cipv_obstacle.pos_s)

    def plot_frenet_obstalce(self, obstalce_list):
        s_list =[]
        d_list =[]
        for obstalce_ in obstalce_list.obstacle:
            s_list.append(obstalce_.pos_s)  
            d_list.append(obstalce_.pos_d * -1)
        self.line3_2.set_xdata(d_list)
        self.line3_2.set_ydata(s_list)

def line_offset(line_o,offset):
    line_t = copy.deepcopy(line_o)

    for i in range(len(line_o.path_node)):
        x_ =line_o.path_node[i].flat_x + offset * np.cos((line_o.path_node[i].heading) * np.pi/180) 
        y_ =line_o.path_node[i].flat_y - offset * np.sin((line_o.path_node[i].heading) * np.pi/180)
        line_t.path_node[i].flat_x =x_
        line_t.path_node[i].flat_y =y_
        # # print("car_x: %f, car_y: %f"%(x_,y_))
        # print("orcar_x: %f, orcar_y: %f"%(line_o.path_node[i].car_x,line_o.path_node[i].car_y))
        print("line_o.path_node[i].car_x: %f"%line_o.path_node[i].flat_x)
        print("offset * np.cos(line_o.path_node[i].heading) %f"% (offset*np.cos(line_o.path_node[i].heading)))
        print("x_: %f"%x_)
    return line_t

def flat_to_vehicle(line_o,loc):
    line_t = copy.deepcopy(line_o)

    a = loc.flat_X
    b = loc.flat_Y
    t = -loc.Heading * np.pi/180
    for i in range(len(line_o.path_node)):
        line_t.path_node[i].car_x = (line_o.path_node[i].flat_x - a)* np.cos(t)   + (line_o.path_node[i].flat_y - b)* np.sin(t)
        line_t.path_node[i].car_y = (line_o.path_node[i].flat_x - a)* -np.sin(t)  + (line_o.path_node[i].flat_y - b)* np.cos(t)
    return line_t

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


class Location:
    def __init__(self):
        self.flat_X=0
        self.flat_Y=0
        self.flat_Heading=0


if __name__ == "__main__":
    file_pb = read_protobuf_file("../data/data_convert") 
    p = Plot()
    l_line = proto.obstacleSel_pb2.Path()
    r_line = proto.obstacleSel_pb2.Path()
    lane_width = 3.6
    location = Location()
    location.flat_X = -65.2941812771520	
    location.flat_Y = -171.744892903921
    location.Heading = 270.400597767095

    l_line = line_offset(file_pb.frame[11].path,  -lane_width/2)
    r_line = line_offset(file_pb.frame[11].path,  lane_width/2)
    l_line= flat_to_vehicle(l_line,location)
    r_line= flat_to_vehicle(r_line,location)

    print(file_pb.frame_total_num)
    
    for idx in range(file_pb.frame_total_num):
        # print(idx)
        if(idx < 400):
            continue
        p.plot_vehicle_lane(l_line,r_line)
        # p.plot_vehicle_path(file_pb.frame[11].path)
        idx =555
        p.plot_flat_path(file_pb.frame[11].path)
        p.plot_vehicle_obstalce(file_pb.frame[idx].obstacle_list, file_pb.frame[idx].cipv_obj)
        p.plot_frenet_obstalce(file_pb.frame[idx].obstacle_list)
        p.plot_frenet_path(file_pb.frame[11].path)
        p.plot_frenet_lane(file_pb.frame[11].path,lane_width,file_pb.frame[idx].cipv_obj)
        plt.pause(0.1)







        
