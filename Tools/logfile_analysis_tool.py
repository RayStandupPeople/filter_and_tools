#!/usr/bin/python
#coding=utf-8

# /**  
#  * All rights Reserved, Designed By BAIC
#  * @Title:  logfile_analysis_tool.py 
#  * @Description:  For personal usage, Analysis Logged Data and visualization 
#  * @author: limin.zhu    
#  * @date:   2020.09.29  
#  * @version V1.0 
#  * @Copyright: ${2020} www.baicgroup.com.cn Inc. All rights reserved. 
#  */  

import numpy as np 
from matplotlib import pyplot as plt 
import time

PLAY_SPEED = 0.1  # sapmle time , unit:second
class Obstacle:
    '''
    @breaf Obstacle struct class
    '''
    def __init__(self):
        self.obj_id = 0
        self.obj_type = 0
        self.obj_x = 0
        self.obj_y = 0

class Frame:
    '''
    @breaf Frame Struct class
    '''
    def __init__(self):
        self.current_frame_num = 0
        self.obstacle_num = 0
        self.obstacle_list = []

class Plot(Obstacle):
    '''
    @breaf display tool which help to deal with log file
    which can do these:
    1. parse log file to get all relevent objects info with all frame
    2. display object position at ith frame, dynamic display continuous frames 
    3. display your interested obstacle info by ID 
    4. disPlay the difference between two list
    '''
    def __init__(self,file_name):
        self.f_name = file_name
        self.frame_list = []
        self.target_ID_list =[]
        self.target_TYPE_list =[]  # TEMP  FOR TEMPFUN  get_target_object_list_with_type
        self.grid = plt.GridSpec(6,4,wspace=0.5,hspace=0.5)  # figure window distribute rule
        self.total_frame_num =0  
        self.sample_time = 0.1  # unit: second
        plt.suptitle("Log Analysis and Visualization",fontsize=15)
        
    def show_object_lists(self,frame_num):
        '''
        @breaf show object lists content
        @param frame_num: which frame to be shown 
        '''
        obj_id_ = []  
        # obj_type_ = []     
        obj_x_=[]
        obj_y_=[]
        obj_y__=[]
        obj_x__=[]
    
        if frame_num <= 0:
            frame_num=0

        for object_idx_ in self.frame_list[frame_num].obstacle_list:
            obj_id_.append(object_idx_.obj_id)
            # obj_y_.append(object_idx_.obj_type)
            obj_x_.append(object_idx_.obj_x)
            obj_y_.append(object_idx_.obj_y)
        for i in obj_y_:
            obj_y__.append(i*-1) 
        for i in obj_x_:
            obj_x__.append(i+0.3) 

        # display config
        # plt.cla()  # clear figure
        ax_=plt.subplot(self.grid[0:3,3])
        ax_.cla()
        lane_width =3.6
        ax_.plot([-0.5*lane_width,-0.5*lane_width],[-20,100],"r")  #TEMP leftline
        ax_.plot([0.5*lane_width,0.5*lane_width],[-20,100],'r')    #TEMP rightline
        ax_.plot([-1.5*lane_width,-1.5*lane_width],[-20,100],"black")  #TEMP left-leftline
        ax_.plot([1.5*lane_width,1.5*lane_width],[-20,100],'black')    #TEMP right-rightline
        # ax_.plot(9.5,38,'bo')    #TEMP Point Indi
        # ax_.plot(9.5,35,'ro')    #TEMP Point Indi
        # ax_.text(10,37.5,":Vehicle",fontsize=10)
        # ax_.text(10,34.5,":Pedestrian",fontsize=10)
        ax_.plot(obj_y__,obj_x_,"o")
        for (x,y,id_) in zip(obj_x__,obj_y__,obj_id_):
            plt.text(y,x,str(id_),fontsize=8)
        # print("** Frame %d Obstacles Position Show **" %(frame_num))
        ax_.axis([-10,10,-10,40])
        # plt.xlabel("Y Axis")
        plt.ylabel("X Axis")
        # plt.title("Obstacles - Vehicle Coordinate",fontweight=20)


    def show_global_info(self,cur_frame_num=0,tol_frame_num=0):
        '''
        @breaf show global information 
        @param cur_frame_num: the num of current frame
        @param tol_frame_num: the num of total frame
        '''   
        # display config
        # plt.cla()  # clear figure
        ax_=plt.subplot(self.grid[3:6,3])
        ax_.cla()
        ax_.plot([tol_frame_num]*2,[9.5,10],'r') #total frame number line
        ax_.text(tol_frame_num+1,9.5,"F-Tol: "+str(tol_frame_num),fontsize=10)
        ax_.plot(cur_frame_num % 150,9.5,'r*',markersize="10") #cur frame pointer, jump to origin posi when hit boundary of plot  
        ax_.text((cur_frame_num-2) % 150,9,"F-Cur/Tol: "+str(cur_frame_num)+" / "+str(tol_frame_num),fontsize=10)
        ax_.plot([0,cur_frame_num],[9.5,9.5],'r') #frame readed line

        ax_.text(1,8,"Cur time / Tol time(second):",fontsize=9)
        ax_.text(1,7.5," ->   "+ str(cur_frame_num * self.sample_time) + " / " + str(tol_frame_num * self.sample_time), fontsize=10)
        ax_.text(1,7,"Log File name : ", fontsize=10)
        ax_.text(1,6.5," ->   "+ self.f_name, fontsize=7)
        ax_.text(1,6,"Play Speed : ", fontsize=10)
        ax_.text(1,5.5," ->   "+ str(PLAY_SPEED) + ' second', fontsize=10)
        ax_.axis([0,150,0,10])
        plt.xticks([])
        plt.yticks([])
        plt.xlabel("Global Info")



    def show_target_obstacle(self, target_id):
        '''
        @breaf show your interested obstacle info by ID
        @param target_id: id of your interested obs
        @para wind_pos: which pos in Show Target elements Window, should in 0~4
        '''
        if self.get_target_object_list(target_id)==0: # can not find tar by id
            print("ERROR")
            return
        with open("../LOG/data_convert",mode="w") as f:
            self.show_target_element(f,"obj_id",[0,1],0)
            self.show_target_element(f,"obj_type",[1,2],0)
            self.show_target_element(f,"obj_x",[2,4],1)
            self.show_target_element(f,"obj_y",[4,6],1)

    def show_target_element(self, file_write, element_type, wind_pos=[0,1], flg_print2file = 0):
        '''
        @breaf display the element of target obstacle
        @para file_write: handle of file which to be written
        @para element_type: "obj_id", "obj_type", "obj_x", "obj_y"..and so on
        @para wind_pos: which pos in Show Target elements Window with start and end, like [0,3] [2,3]
        @para flg_print2file: flag, which if print related data to file
        '''
        time_stamp_num = len(self.target_ID_list)
        # time_stamp_num = len(self.target_TYPE_list)
        t=np.linspace(0, time_stamp_num-1,time_stamp_num)
        element_val_list=[]
        for tgt in self.target_ID_list:
        # for tgt in self.target_TYPE_list:
            class_num_opra = eval("tgt."+element_type)
            element_val_list.append(class_num_opra)
        
        ## file Print
        if flg_print2file ==1:
            for ele in element_val_list:
                file_write.write(str(ele) + " ")
            file_write.write("\n")
        ##  # display config
        s=wind_pos[0]
        e=wind_pos[1]
        plt.subplot(self.grid[s:e,0:3])
        s=wind_pos[0]
        e=wind_pos[1]
        ax_=plt.subplot(self.grid[s:e,0:3])
        if cur_frame ==0:
            ax_.cla()
        ax_.plot(t,element_val_list,'bo-')
        plt.xlabel("Stamp No.")
        plt.ylabel(element_type)
        # plt.xlim([0, self.total_frame_num * self.sample_time])
        

    def show_obstacle_compare(self):
        plt.subplot(self.grid[0:3,0:3])
        plt.subplot(self.grid[3:6,0:3])


    def read_data_from_file(self):
        '''
        @breaf get object list with all frame times from data file
        @param none
        @return object lists number
        '''
        with open(self.f_name) as f:
            line = f.readline()
            while True:   
                if(line.find("R=obstacle_num")!=-1):    # find new frame
                    frame = Frame()
                    while True:
                        if(line.find("=888=R=Obstacles[0].id")!=-1):     # find new Obstacle 
                            obj =Obstacle() 
                            obj.obj_id = int(line.split("=")[-1].strip())
                            line = f.readline()
                            while True:               # assign left properties of this frame
                                if(line.find("=888=R=Obstacles[0].type")!=-1):
                                    obj.obj_type = float(line.split("=")[-1].strip())
                                if(line.find("=888=R=Obstacles[0].pos_x")!=-1):
                                    obj.obj_x = float(line.split("=")[-1].strip())
                                if(line.find("=888=R=Obstacles[0].pos_y")!=-1):
                                    obj.obj_y = float(line.split("=")[-1].strip())

                                if(line.find("obstacle_num")!=-1):    #exit condition1, new frame
                                    break
                                if(line.find("=888=R=Obstacles[0].id")!=-1):             #exit condition2, new obstacle
                                    break
                                line = f.readline()
                                if not line:                          #exit condition3, EOF
                                    break
                            frame.obstacle_list.append(obj)
                            if(line.find("R=obstacle_num")!=-1):    #exit condition1, new frame
                                break
                        else:
                            line = f.readline()
                            if not line:
                                break
                    frame.current_frame_num = self.total_frame_num
                    self.frame_list.append(frame)# save frame to frame_list
                    self.total_frame_num+=1
                    # print("frame size: %d" %len(frame))
                else:
                    line = f.readline()
                    if not line:
                        break
        return self.total_frame_num

                    
    def get_target_object_list(self, target_id=-1):
        '''
        @breaf get target object list with all frame times by target id
        @param target_id: the interested id of target
        @return the show time of interested target 
        '''
        if target_id == -1:
            print("** ERROR: please input your interested obstacle ID **")
            return 
        #find targetID by all frames
        for frame_idx_ in self.frame_list:    
            for obstalce_idx_ in frame_idx_.obstacle_list:
                if(obstalce_idx_.obj_id == target_id):
                    self.target_ID_list.append(obstalce_idx_) 
        target_obstalce_num = len(self.target_ID_list) 
        if target_obstalce_num == 0:
            print("** ERROR: can not find target obstacle with ID %d **" %target_id)
        else:
            print("** The num of Frame with Target ID is %d **" %len(self.target_ID_list))    
        return len(self.target_ID_list)  

    # # TEMP FUN: which get interested taget with type     
    # def get_target_object_list_with_type(self, target_type=-1):
    #     '''
    #     @breaf get target object list with all frame times by target type
    #     @param target_type: the interested type of target
    #     @return the show time of interested target 
    #     '''
    #     if target_type == -1:
    #         print("** ERROR: please input your interested obstacle TYPE **")
    #         return 
    #     #find targetID by all frames
    #     for frame_idx_ in self.frame_list:    
    #         for obstalce_idx_ in frame_idx_.obstacle_list:
    #             if(obstalce_idx_.obj_type == target_type):
    #                 self.target_TYPE_list.append(obstalce_idx_) 
    #     target_obstalce_num = len(self.target_TYPE_list) 
    #     if target_obstalce_num == 0:
    #         print("** ERROR: can not find target obstacle with Type %d **" %target_type)
    #     else:
    #         print("** The num of Frame with Target TYPE is %d **" %len(self.target_TYPE_list))    
    #     return len(self.target_TYPE_list)


    # # TEMP FUN: related with  FUN get_target_object_list_with_type
    # def show_target_obstacle_with_type(self, target_type):
    #     '''
    #     @breaf show your interested obstacle info by TYPE
    #     @param target_type: type of your interested obs
    #     @para wind_pos: which pos in Show Target elements Window, should in 0~4
    #     '''
    #     if self.get_target_object_list_with_type(target_type)==0: # can not find tar by type
    #         print("ERROR")
    #         return
    #     self.show_target_element("obj_id",[0,1])
    #     self.show_target_element("obj_type",[1,2])
    #     self.show_target_element("obj_x",[2,4])
    #     self.show_target_element("obj_y",[4,6])
      

if __name__ == "__main__":
    p = Plot("../LOG/log_2020_1019/Person_move2StaicCar_X.log")
    frame_len = p.read_data_from_file()
    cur_frame=0
    p.show_target_obstacle(17) #input Interested ID  ### Person_move2StaicCar_X....ID 17\215
    # p.show_target_obstacle_with_type(0) #TEMP FUN input Interested TYPE
   
    while True:
        if(cur_frame==frame_len):
            cur_frame=0
        p.show_object_lists(cur_frame)
        p.show_global_info(cur_frame,frame_len)
        cur_frame=cur_frame+1
        # p.show_obstacle_compare()
        plt.pause(PLAY_SPEED)  #play speed
