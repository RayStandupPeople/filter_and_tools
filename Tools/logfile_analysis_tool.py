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

from sys import argv
import numpy as np 
from matplotlib import pyplot as plt 
import time
import platform
import os
import threading

PROTOBUF =0

if PROTOBUF ==1 :
    import libs.types_pb2  #Protobuf environment model data types



PLAY_SPEED = 0.1   # sapmle time , unit:second
PLATFORM_sys = 0   # 0->linux   1->windows

class UserInfo:
    '''
    @breaf temp info class
    '''
    def __init__(self):
        self.obj_mid_flag = 0
        self.obj_left_flag = 0
        self.obj_right_flag =0
        self.obj_mid_s_m  =0
        self.obj_left_s_l =0
        self.obj_right_s_r=0
        self.selected_obj_x=0
        self.selected_obj_y=0
        self.selected_obj_id=0
        

class StatusMachine:
    '''
    @breaf status Machine class
    '''
    def __init__(self):
        self.avp_req = 0
        self.TBOX_AVPModKey = 0

class VehicleSts:
    '''
    @breaf vehicle status class
    '''
    def __init__(self):
        self.vehicle_speed = 0
        self.steerwheel_angle = 0
    

class Localization:
    '''
    @breaf localization class
    '''
    def __init__(self):
        self.lat = 0
        self.lon = 0
        self.latRest_X = 0
        self.lonRest_Y = 0

class HDMapInfo:
    '''
    @breaf Hdmap class
    '''
    def __init__(self):
        self.x = []
        self.y = []
        self.onpath = 0
        self.decision_command =0

class Obstacle:
    '''
    @breaf Obstacle struct class
    '''
    def __init__(self):
        self.obj_id = 0
        self.obj_type = 0
        self.obj_x = 0
        self.obj_y = 0
        self.obj_rel_speed_x = 0
        self.obj_rel_speed_y = 0
        self.obj_abs_speed_x = 0
        self.obj_abs_speed_y = 0
        self.obj_heading = 0
        self.obj_length = 0
        self.obj_width = 0

class Frame:
    '''
    @breaf Frame Struct class
    '''
    def __init__(self):
        self.current_frame_num = 0
        self.logfile_frame_num =0
        self.obstacle_num = 0
        self.obstacle_list = []
        self.localization = Localization()
        self.hdmapinfo = HDMapInfo()
        self.vehiclests = VehicleSts()
        self.statusmachine = StatusMachine()
        self.userinfo = UserInfo()

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
        if PROTOBUF ==1 :
            self.logfile_pb =libs.types_pb2.LogFile()  #logfile data type handler
        plt.suptitle("Log Analysis and Visualization",fontsize=15) 
        if PLATFORM_sys==1:      #windows
            pass
            # plt.get_current_fig_manager().full_screen_toggle()
        else:
            manager = plt.get_current_fig_manager()       #set max window
            manager.resize(*manager.window.maxsize())


    def set_wholeFile_info_to_protobuf(self):
        '''
        @breaf help to put whole file info to protobuf file
        '''  
        self.logfile_pb.frame_num = self.total_frame_num
        for frame in self.frame_list:
            frame_pb = self.logfile_pb.frame.add()      # new frame
            frame_pb.id = frame.current_frame_num

            # set obsatacle_list
            for obstacle_ in frame.obstacle_list:
                obstacle_pb = frame_pb.obstacle.add()   # new obstacle
                obstacle_pb.id = obstacle_.obj_id
                obstacle_pb.pos_x = obstacle_.obj_x
                obstacle_pb.pos_y = obstacle_.obj_y
            
            # set localization
            loc_ = frame.localization
            loc_pb = frame_pb.localization         
            loc_pb.lat = loc_.lat
            loc_pb.lon = loc_.lon
            loc_pb.latRest_X = loc_.latRest_X
            loc_pb.lonRest_Y = loc_.lonRest_Y

            # set HDMap
            hdmap_ = frame.hdmapinfo
            hdmap_pb = frame_pb.hdmapinfo        
            hdmap_pb.onpath = hdmap_.onpath

            # set VehicleSts
            vehiclests_ = frame.vehiclests
            vehiclests_pb = frame_pb.vehiclests
            vehiclests_pb.vehicle_speed = vehiclests_.vehicle_speed


            # set StatusMachine
            statusmachine_ = frame.statusmachine
            statusmachine_pb = frame_pb.statusmachine   
            statusmachine_pb.avp_req = statusmachine_.avp_req
            statusmachine_pb.TBOX_AVPModKey = statusmachine_.TBOX_AVPModKey

            # set User
            userinfo_ = frame.userinfo
            userinfo_pb = frame_pb.userinfo   
            userinfo_pb.obj_left_flag  = userinfo_.obj_left_flag
            userinfo_pb.obj_mid_flag   = userinfo_.obj_mid_flag
            userinfo_pb.obj_right_flag = userinfo_.obj_right_flag
            userinfo_pb.obj_left_s_l   = userinfo_.obj_left_s_l
            userinfo_pb.obj_mid_s_m    = userinfo_.obj_mid_s_m 
            userinfo_pb.obj_right_s_r  = userinfo_.obj_right_s_r 
            
        logfile_string_pb = self.logfile_pb.SerializeToString()
        with open("log/wholeFile_info_pb",'wb') as file_obs_list:
            file_obs_list.write(logfile_string_pb)
            
    def show_global_path(self):
        '''
        @breaf show global path (blue valley right lane)
        '''

        global_path_x =[-120, -30, -20, -15, -10, -10, -13, -30, -115]
        global_path_y =[-30, -30, -28, -25, -20, 0, 6, 10, 10]
        gx_=plt.subplot(self.grid[2:3,3])
        gx_.plot(global_path_x, global_path_y)

    def show_localization(self,frame_num):
            '''
            @breaf show current vehicle localization 
            @param frame_num: which frame to be shown 
            '''

            lx_=plt.subplot(self.grid[2:3,3])
            plt.ylim([-35,15])
            line, = lx_.plot(0,0,'ro',markersize=2)
            # lx_.plot(self.frame_list[frame_num].localization.latRest_X,self.frame_list[frame_num].localization.lonRest_Y,"ro")
            line.set_xdata(self.frame_list[frame_num].localization.latRest_X)
            line.set_ydata(self.frame_list[frame_num].localization.lonRest_Y)
            


    def show_object_lists(self,frame_num):
        '''
        @breaf show object lists content
        @param frame_num: which frame to be shown 
        '''
        obj_id_ = []  
        obj_type_ = []     
        obj_x_=[]
        obj_y_=[]
        obj_y__=[]
        obj_x__=[]
        type_list = ['ped','veh','oth']
    
        if frame_num <= 0:
            frame_num=0

        for object_idx_ in self.frame_list[frame_num].obstacle_list:
            obj_id_.append(object_idx_.obj_id)
            obj_type_.append(object_idx_.obj_type)
            obj_x_.append(object_idx_.obj_x)
            obj_y_.append(object_idx_.obj_y)
        # plot origin point    
        obj_x_.append(0)
        obj_y_.append(0)
        for i in obj_y_:
            obj_y__.append(i*-1) 
        for i in obj_x_:
            obj_x__.append(i+0.3) 

        # display config
        # plt.cla()  # clear figure
        ax_=plt.subplot(self.grid[0:2,3])
        ax_.cla()
        lane_width = 2.7
        ax_.plot([-0.5*lane_width,-0.5*lane_width],[-20,100],"r")  #TEMP leftline
        ax_.plot([0.5*lane_width,0.5*lane_width],[-20,100],'r')    #TEMP rightline
        ax_.plot([-1.5*lane_width,-1.5*lane_width],[-20,100],"black")  #TEMP left-leftline
        ax_.plot([1.5*lane_width,1.5*lane_width],[-20,100],'black')    #TEMP right-rightline
        # ax_.plot(9.5,38,'bo')    #TEMP Point Indi
        # ax_.plot(9.5,35,'ro')    #TEMP Point Indi
        # ax_.text(10,37.5,":Vehicle",fontsize=10)
        # ax_.text(10,34.5,":Pedestrian",fontsize=10)
        ax_.plot(obj_y__,obj_x_,"o")
        for (x,y,id_,type_) in zip(obj_x__,obj_y__,obj_id_,obj_type_):   # pack all property into tuple
            plt.text(y,x,str(id_) +" " + type_list[int(type_)],fontsize=8)
        # print("** Frame %d Obstacles Position Show **" %(frame_num))
        ax_.axis([-10,10,-20,40])
        # plt.xlabel("Y Axis")
        plt.ylabel("X Axis")
        # plt.title("Obstacles - Vehicle Coordinate",fontweight=20)



        # LKW Temp S  
        user_  = self.frame_list[frame_num].userinfo
        obj_y_.append(lane_width* -1)
        if user_.obj_left_s_l >0:
            obj_x_.append(user_.obj_left_s_l)
        else:
            obj_x_.append(-100) 

        obj_y_.append(0)
        if user_.obj_mid_s_m >0:
            obj_x_.append(user_.obj_mid_s_m)
        else:
            obj_x_.append(-100)  
        obj_y_.append(lane_width)
        if user_.obj_right_s_r >0:
            obj_x_.append(user_.obj_right_s_r)
        else:
            obj_x_.append(-100) 

        #ax_.plot(obj_y_,obj_x_,"o")
        # LKW TEMP E



        # ZLM selected OBJ Plot  START
        if user_.selected_obj_x >0:
            ax_.plot(user_.selected_obj_y*-1,user_.selected_obj_x,'ro')
      
        # ZLM selected OBJ Plot  END

    def show_global_info(self,cur_frame_num=0,tol_frame_num=0):
        '''
        @breaf show global information 
        @param cur_frame_num: the num of current frame
        @param tol_frame_num: the num of total frame
        '''   
        # display config
        # plt.cla()  # clear figure
        pos_idx =self.f_name.find("zlm_")   #to reduce the len of file name by key
        if pos_idx==-1:
            file_name = self.f_name
        else:
            file_name=self.f_name[pos_idx:]
        ax_=plt.subplot(self.grid[3:6,3])
        ax_.cla()
        ax_.plot([tol_frame_num]*2,[9.5,10],'r') #total frame number line
        ax_.text(tol_frame_num+1,9.5,"F-Tol: "+str(tol_frame_num),fontsize=10)
        ax_.plot(cur_frame_num % 150,9.5,'r*',markersize="10") #cur frame pointer, jump to origin posi when hit boundary of plot  
        ax_.text((cur_frame_num-2) % 150,9,"F-Cur/Tol: "+str(cur_frame_num)+" / "+str(tol_frame_num),fontsize=10)
        ax_.plot([0,cur_frame_num],[9.5,9.5],'r') #frame readed line

        ax_.text(1,8,"Cur LogFile Frame:",fontsize=9)
        ax_.text(1,7.5," ->   "+ str(self.frame_list[cur_frame_num].logfile_frame_num), fontsize=10)

        ax_.text(1,7,"Log File name : ", fontsize=10)
        ax_.text(1,6.5," ->   "+ file_name, fontsize=7)

        ax_.text(1,6,"Play Speed : ", fontsize=10)
        ax_.text(1,5.5," ->   "+ str(PLAY_SPEED) + ' second', fontsize=10)

        ax_.text(1,5,"Localization : ", fontsize=10)
        ax_.text(1,4.5," ->   "+ str(self.frame_list[cur_frame_num].localization.latRest_X) + \
        " , " + str(self.frame_list[cur_frame_num].localization.lonRest_Y), fontsize=10)
        ax_.text(1,4,"HdmapInfo : ", fontsize=10)
        ax_.text(1,3.5," ->   "+ "Onpath: " + str(self.frame_list[cur_frame_num].hdmapInfo.onpath) , fontsize=10)
        ax_.text(1,3,"VehicleSts : ", fontsize=10)
        ax_.text(1,2.5," ->   "+ "speed: " + str(round(self.frame_list[cur_frame_num].vehiclests.vehicle_speed,2)) + " KPH" , fontsize=10)

        ax_.text(1,2,"User info : ", fontsize=10)
        ax_.text(1,1.5," ->   "+ "left:   " + str(round(self.frame_list[cur_frame_num].userinfo.obj_left_s_l, 2)) + \
             "  mid:  " + str(round(self.frame_list[cur_frame_num].userinfo.obj_mid_s_m, 2)) + "   right  :  " + \
             str(round(self.frame_list[cur_frame_num].userinfo.obj_right_s_r, 2)), fontsize=10)
        
        ax_.text(1,1," ->   "+ "decision command:  " + str(self.frame_list[cur_frame_num].hdmapInfo.decision_command), fontsize=10)

        ax_.axis([0,150,0,10])
        plt.xticks([])
        plt.yticks([])
        plt.xlabel("Global Info")



    def show_target_obstacle(self, target_id=[]):
        '''
        @breaf show your interested obstacle info by ID
        @param target_id: id of your interested obs
        @para wind_pos: which pos in Show Target elements Window, should in 0~4
        '''
        if self.get_target_object_list(target_id)==0: # can not find tar by id
            # print("ERROR: CAN NOT FIND target by ID")
            return
        
        time_stamp_num = len(self.target_ID_list)
        t=np.linspace(0, time_stamp_num-1,time_stamp_num)
        logfile_frame_val_list=[]
        for frame_ in self.frame_list:
            logfile_frame_val_list.append(frame_.logfile_frame_num)
        lf_f = plt.subplot(self.grid[0:1,0:3])
        lf_f.plot(t,logfile_frame_val_list)
        plt.xlabel("Stamp No.")
        plt.ylabel("Logfile Frame No")

        # show target obstacle property    
        
        ## xhw ###
        '''
        @element List:
        obj_id
        obj_type
        obj_x
        obj_y
        obj_rel_speed_x
        obj_rel_speed_y
        obj_abs_speed_x
        obj_abs_speed_y
        obj_heading
        obj_length
        obj_width 

        selected_obj_x
        selected_obj_y
        selected_obj_id
        '''
        self.show_target_element("obj_id",[1,2])
        self.show_target_element("obj_rel_speed_x",[2,4])
        self.show_target_element("obj_heading",[4,6])
        ## xhw ###

        
    def show_target_element(self, element_type, wind_pos=[0,1]):
        '''
        @breaf display the element of target obstacle
        @para file_write: handle of file which to be written
        @para element_type: "obj_id", "obj_type", "obj_x", "obj_y"..and so on
        @para wind_pos: which pos in Show Target elements Window with start and end, like [0,3] [2,3]
        @para flg_print2file: flag, which if print related data to file
        '''
        time_stamp_num = len(self.frame_list)
        # time_stamp_num = len(self.target_TYPE_list)
        t=np.linspace(0, time_stamp_num-1,time_stamp_num)
        element_val_list=[]
        if not element_type.find('selected_obj'):
            for frame_ in self.frame_list:
                try:
                    class_num_opra = eval("frame_.userinfo."+element_type)
                    element_val_list.append(class_num_opra)
                except AttributeError:
                    element_val_list.append(0)
        else:
            for tgt in self.target_ID_list:
                try:
                    class_num_opra = eval("tgt."+element_type)
                    element_val_list.append(class_num_opra)
                except AttributeError:
                    element_val_list.append(0)

            
        ##  # display config
        s=wind_pos[0]
        e=wind_pos[1]
        plt.subplot(self.grid[s:e,0:3])
        s=wind_pos[0]
        e=wind_pos[1]
        ax_=plt.subplot(self.grid[s:e,0:3])
        if global_cur_frame ==0:
            ax_.cla()
        ax_.plot(t,element_val_list)
        plt.xlabel("Stamp No.")
        plt.ylabel(element_type)
        # plt.xlim([0, self.total_frame_num * self.sample_time])
        



    def clean_and_check_data(self, line, data_type):
        '''
        @breaf clean data and check if sucess, if not then fix it with 0
        @param line: original string line data
        @param data_type: the data type need to trans to
        @return needed value
        '''
        data_type_ = eval(data_type)
        try:
            val = data_type_(line.split("=")[-1].strip())
        except ValueError:
            print("warning: parse line failed %s"%(line))
            val = 0
        return val

    def check_line_need_break(self,line,f):
        '''
        @breaf check if current line meet break condition, 0->no need, 1 ->need
        @param line: original string line data
        @param f: file handler
        @return break flag   0->no need, 1 ->need
        '''
        if(line.find("R=frame_index")!=-1):                #exit condition
            return 1
        if(line.find("=888=R=Obstacles[0].id")!=-1):       #exit condition
            return 1
        if(line.find("R=Latitude")!=-1):                   #exit condition
            return 1
        if(line.find("R=vehicleInfo.ESP_VehSpd")!=-1):     #exit condition
            return 1
        if(line.find("R=hdmapInfo.planpath")!=-1):         #exit condition
            return 1
        if(line.find("coll_obj.obj_mid_flag")!=-1):        #exit condition
            return 1
        return 0

    def parse_frameInfo_from_file(self):
        '''
        @breaf get object list with all frame times from data file
        @param none
        @return object lists number
        '''
        with open(self.f_name) as f:
            line = f.readline()
            while True:   
                if(line.find("R=frame_index")!=-1):    # find new frame
                    frame = Frame()
                    frame.logfile_frame_num = self.clean_and_check_data(line,'int')
                    frame_obs_num=0
                    while True:
                        #####              Obtatacle List         ####
                        if(line.find("=888=R=Obstacles[0].id")!=-1):     # find new Obstacle 
                            obj =Obstacle() 
                            frame_obs_num = frame_obs_num +1
                            obj.obj_id = self.clean_and_check_data(line,'int')
                            line = f.readline()
                            while True:               # assign left properties of this frame
                                if(line.find("=888=R=Obstacles[0].type")!=-1):
                                    obj.obj_type = self.clean_and_check_data(line,'float')
                                if(line.find("=888=R=Obstacles[0].pos_x")!=-1):
                                    obj.obj_x = self.clean_and_check_data(line,'float')
                                if(line.find("=888=R=Obstacles[0].pos_y")!=-1):
                                    obj.obj_y = self.clean_and_check_data(line,'float')
                                if(line.find("=888=R=Obstacles[0].rel_speed_x")!=-1):
                                    obj.obj_rel_speed_x = self.clean_and_check_data(line,'float')
                                if(line.find("=888=R=Obstacles[0].rel_speed_y")!=-1):
                                    obj.obj_rel_speed_y = self.clean_and_check_data(line,'float')
                                if(line.find("=888=R=Obstacles[0].abs_speed_x")!=-1):
                                    obj.obj_abs_speed_x = self.clean_and_check_data(line,'float')
                                if(line.find("=888=R=Obstacles[0].abs_speed_y")!=-1):
                                    obj.obj_abs_speed_y = self.clean_and_check_data(line,'float')
                                if(line.find("=888=R=Obstacles[0].heading")!=-1):
                                    obj.obj_heading = self.clean_and_check_data(line,'float')
                                if(line.find("=888=R=Obstacles[0].length")!=-1):
                                    obj.obj_length = self.clean_and_check_data(line,'float')
                                if(line.find("=888=R=Obstacles[0].width")!=-1):
                                    obj.obj_width  = self.clean_and_check_data(line,'float')     
                         
                                if self.check_line_need_break(line,f):
                                    break
                                line = f.readline()
                                if not line:                                       #exit condition, EOF
                                    break
                            frame.obstacle_list.append(obj)
                            if(line.find("R=frame_index")!=-1):    #exit condition1, new frame
                                break


                                #####               Localization         ####
                        if(line.find("R=Latitude")!=-1):     # find new Locallization
                            loclizon=Localization()
                            loclizon.lat = self.clean_and_check_data(line,'float')
                            line = f.readline()
                            while True:               # assign left properties of this frame
                                if(line.find("R=Longitude")!=-1):
                                    loclizon.lon= self.clean_and_check_data(line,'float')
                                if(line.find("R=LocalizationResult.x")!=-1):
                                    loclizon.latRest_X = self.clean_and_check_data(line,'float')
                                if(line.find("R=LocalizationResult.y")!=-1):
                                    loclizon.lonRest_Y = self.clean_and_check_data(line,'float')

                                if self.check_line_need_break(line,f):
                                    break
                                line = f.readline()
                                if not line:                          #exit condition3, EOF
                                    break
                            frame.localization = loclizon
                            if(line.find("R=frame_index")!=-1):    #exit condition1, new frame
                                break

                                #####               Vehicle Status         ####
                        if(line.find("R=vehicleInfo.ESP_VehSpd")!=-1):     # find new vehicle status
                            vehiclests_=VehicleSts()
                            vehiclests_.vehicle_speed = self.clean_and_check_data(line,'float')
                            line = f.readline()
                            while True:               # assign left properties of this frame
                                if self.check_line_need_break(line,f):
                                    break
                                line = f.readline()
                                if not line:                          #exit condition3, EOF
                                    break
                            frame.vehiclests = vehiclests_
                            if(line.find("R=frame_index")!=-1):    #exit condition1, new frame
                                break


                                #####               Hdmap         ####
                        if(line.find("R=hdmapInfo.planpath")!=-1):     # find new hdmap
                            hdmapinfo_=HDMapInfo()
                            hdmapinfo_.onpath = self.clean_and_check_data(line,'int')
                            line = f.readline()
                            while True:               # assign left properties of this frame
                                if(line.find("=R=decisionToPlan.decision_command")!=-1):
                                    hdmapinfo_.decision_command = self.clean_and_check_data(line,'int')
                                if self.check_line_need_break(line,f):
                                    break
                                line = f.readline()
                                if not line:                          #exit condition3, EOF
                                    break
                            frame.hdmapInfo = hdmapinfo_
                            if(line.find("R=frame_index")!=-1):    #exit condition1, new frame
                                break

                                #####               User info         ####
                        if(line.find("coll_obj.obj_mid_flag")!=-1):     # find new user info
                            userinfo_=UserInfo()
                            userinfo_.obj_mid_s_m = self.clean_and_check_data(line,'float')
                            if userinfo_.obj_mid_s_m >0 :
                                userinfo_.obj_mid_flag = 1
                            line = f.readline()
                            while True:               # assign left properties of this frame
                                if(line.find("coll_obj.obj_lef_flag")!=-1):
                                    userinfo_.obj_left_s_l = self.clean_and_check_data(line,'float')
                                    if userinfo_.obj_left_s_l >0:
                                        userinfo_.obj_left_flag = 1

                                if(line.find("coll_obj.obj_rig_flag")!=-1):
                                    userinfo_.obj_right_s_r = self.clean_and_check_data(line,'float')
                                    if userinfo_.obj_right_s_r >0 :
                                        userinfo_.obj_right_flag = 1

                                #  input your interested key words #
                               
                                if(line.find("selectObj->frontMid.obj.s")!=-1):
                                    userinfo_.selected_obj_x = self.clean_and_check_data(line,'float')
                                if(line.find("selectObj->frontMid.obj.pos_y")!=-1):
                                    userinfo_.selected_obj_y = self.clean_and_check_data(line,'float')
                                if(line.find("selectObj->frontMid.obj.id")!=-1):
                                    userinfo_.selected_obj_id = self.clean_and_check_data(line,'int')
                                   
                                #  input your interested key words #
                                if self.check_line_need_break(line,f):
                                    break

                                line = f.readline()
                                if not line:                          #exit condition3, EOF
                                    break
                            frame.userinfo = userinfo_
                            if(line.find("R=frame_index")!=-1):    #exit condition1, new frame
                                break



                                #####               Read next         ####
                        else:
                            line = f.readline()
                            if not line:
                                break
                            if(line.find("R=frame_index")!=-1):    # find new frame
                                break


                    frame.current_frame_num = self.total_frame_num
                    frame.obstacle_num =frame_obs_num
                    
                    self.frame_list.append(frame)# save frame to frame_list
                    self.total_frame_num+=1
                    # print("frame size: %d" %len(frame))
                else:
                    line = f.readline()
                    if not line:
                        break
        return self.total_frame_num

                    
    def get_target_object_list(self, target_id_list_=-1):
        '''
        @breaf get target object list with all frame times by target id
        @param target_id: the interested id of target
        @return the show time of interested target 
        '''
        if target_id_list_ == -1:
            if(len(self.frame_list)<10 or len(self.frame_list[0].obstacle_list)==0):
                print("Warning: frame or obstacle is EMPTY ")
                return
            print("Warning: you have NOT typied in your interested obstacle ID")
            target_id_list_ = self.frame_list[0].obstacle_list[0].obj_id
        #find targetID by all frames
        for frame_idx_ in self.frame_list:
            exist_match_id =0    
            for obstalce_idx_ in frame_idx_.obstacle_list:
                if(target_id_list_ == obstalce_idx_.obj_id ):
                    self.target_ID_list.append(obstalce_idx_) 
                    exist_match_id =1
            if not exist_match_id:
                self.target_ID_list.append(0) 
                
        target_obstalce_num = len(self.target_ID_list) 
        if target_obstalce_num == 0:
            print("ERROR: can not find target obstacle with ID and so on..%d " %target_id_list_[0])
        return len(self.target_ID_list)  

class myThread (threading.Thread):
    def __init__(self, threadID, name, counter):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.counter = counter
    def run(self):
        print ("thread begin: " + self.name)
        global global_cur_frame
        while True:
            print ("input the CurrentFrame No you want!")
            arr = input()
            try:
                global_cur_frame = arr
            except SyntaxError:
                print("err input, try again")
            else:
                print("Now->  CurrentFrame: %d"%( global_cur_frame))
            # print(global_cur_frame)
            # if global_cur_frame>450:
            #     global_cur_frame =0
            time.sleep(1)

        print ("thread exit: " + self.name)

if __name__ == "__main__":
    global global_cur_frame
    global global_ID_interested
    if platform.system() == "Windows":
        PLATFORM_sys =1
        # file_dir ="E:/vmShare/LOGFILE/"
        file_dir ="log/log_list/"
    else:
        PLATFORM_sys =0
        # file_dir ="/mnt/hgfs/vmShare/LOGFILE/"
        file_dir ="log/log_list/"

    if len(argv)==1:
        # file_name  = "../log/log_2020_1023/staticCar_perMove.log"         # file name you want to play
        # file_name  = file_dir+"putty1124152404.log"         # file name you want to play
        for (root,dirs,files) in os.walk(file_dir):
            file_name = file_dir + files[-1]
    else:
        print("Warning : read from shared file: " +str(argv[1]))
        file_name = str(argv[1])                                                  # file from outside
    global_cur_frame = 400                                                       # xhw file position you want to play from
    global_ID_interested = 1                                                   # xhw DisPlay the OBSTCAL with the specified ID ARRAY
    p = Plot(file_name)
    print("parsing File......")
    time_s = time.time()
    frame_len = p.parse_frameInfo_from_file()
    print("cost time: %f seconds" %(time.time()-time_s))
    p.show_target_obstacle(global_ID_interested) #input Interested ID  ### Person_move2StaicCar_X....ID 17\215
    # p.set_obstacleList_to_file()
    if PROTOBUF ==1 :
        p.set_wholeFile_info_to_protobuf()
    # p.set_user_to_file()
    p.show_global_path()

    thread1 = myThread(1, "Thread-1", 1)
    thread1.start()


    print("loading......")
    while True:
        if(global_cur_frame==frame_len):
            global_cur_frame=0
        p.show_object_lists(global_cur_frame)
        p.show_global_info(global_cur_frame,frame_len)
        p.show_localization(global_cur_frame)
        global_cur_frame=global_cur_frame+1
        plt.pause(PLAY_SPEED)  #play speed
