#!/usr/bin python

import libs.types_pb2

logfile_pb = libs.types_pb2.LogFile()
# frame_pb = libs.types_pb2.Frame()

# obstacle_pb = frame_pb.obstacle.add()

# print(frame_pb.id)
# serializeToString = frame_pb.SerializeToString()
with open("../log/obstacle_list_info_pb") as file:
    serializeToString = file.read()



logfile_pb.ParseFromString(serializeToString)
print("frame num: "+ str(logfile_pb.frame_num))
for frame_ in logfile_pb.frame:
    print("frame id: "+str(frame_.id))
    
    for obstacle_ in frame_.obstacle:
        print("obs_id: "+str(obstacle_.id))
        print("obs_posx: " + str(obstacle_.pos_x))



