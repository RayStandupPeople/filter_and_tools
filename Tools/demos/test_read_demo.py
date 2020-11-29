#!/usr/bin python

import sys
sys.path.append("..")
import libs.types_pb2

logfile_pb = libs.types_pb2.LogFile()

with open("../log/wholeFile_info_pb",'rb') as file:
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

   




