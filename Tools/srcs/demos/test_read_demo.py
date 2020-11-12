#!/usr/bin python


# message LogFile {
#     int32 frame_num = 1;
#     repeated Frame frame = 2;
# }

# message Frame {
#     int32 id = 1;
#     repeated ObstacleList obstacle_list =2;    
#     Locallization localization = 3;
#     HDMapInfo hdmapinfo = 4;
#     VehicleSts vehiclests = 5;
#     StatusMachine statusmachine = 6;
#     UserInfo userinfo = 7;
# }

# message Locallization {
#     float lat = 1;
#     float lon = 2;
#     float latRest_X = 3;
#     float lonRest_Y = 4;
# }

# message HDMapInfo {
#     int32 x =1;
#     int32 y =2;
#     bool onpath =3;
# }

# message VehicleSts {
#     float vehicle_speed =1;
#     float steerwheel_angle=2;
# }

# message StatusMachine {
#     int32 avp_req = 1;
#     int32 TBOX_AVPModKey = 2;
# }

# message ObstacleList {
#     int32 id = 1;
#     int32 type = 2;
#     float pos_x = 3;
#     float pos_y = 4;
#     float rel_spd_x = 5;
#     float rel_spd_y = 6;
# }

# message UserInfo {
#     bool obj_left_flag  = 1;
#     bool obj_mid_flag   = 2;
#     bool obj_right_flag = 3;
#     float obj_left_s_l  = 4;
#     float obj_mid_s_m   = 5;
#     float obj_right_s_r   = 6;
# }
import sys
sys.path.append("..")
import libs.types_pb2

logfile_pb = libs.types_pb2.LogFile()

with open("../../../log/wholeFile_info_pb") as file:
    fileInfo_pb = file.read()
logfile_pb.ParseFromString(fileInfo_pb)



print("frame num: " + str(logfile_pb.frame_num))
for frame_ in logfile_pb.frame:
    print("frame id: "+str(frame_.id))
    for obstacle_ in frame_.obstacle:
        print("id: "+str(obstacle_.id))
        print("type: " + str(obstacle_.type))
        print("pos_x: " + str(obstacle_.pos_x))
        print("pos_y: " + str(obstacle_.pos_y))

    localization_ = frame_.localization
    print("latRest_X: "+str(localization_.latRest_X))
    print("lonRest_Y: "+str(localization_.lonRest_Y))

    hdmapinfo_ = frame_.hdmapinfo
    print("onpath: "+str(hdmapinfo_.onpath))

    vehiclests_ = frame_.vehiclests
    print("vehicle_speed: "   +str(vehiclests_.vehicle_speed))
    print("steerwheel_angle: "+str(vehiclests_.steerwheel_angle))

   




