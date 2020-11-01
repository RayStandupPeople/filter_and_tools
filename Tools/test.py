#!/usr/bin python

import libs.types_pb2

logfile_pb =libs.types_pb2.LogFile()
logfile_pb.frame_num =20

frame_pb = logfile_pb.frame.add()
frame_pb.id = 1
obstacle_pb = frame_pb.obstacle.add()
obstacle_pb.id =1
obstacle_pb.pos_x = 30.6
obstacle_pb = frame_pb.obstacle.add()
obstacle_pb.id =2
obstacle_pb.pos_x = 10.6

frame_pb = logfile_pb.frame.add()
frame_pb.id = 2
obstacle_pb = frame_pb.obstacle.add()
obstacle_pb.id =3
obstacle_pb.pos_x = 20.6
obstacle_pb = frame_pb.obstacle.add()
obstacle_pb.id =4
obstacle_pb.pos_x = -10.6

# print(frame_pb.id)
serializeToString = logfile_pb.SerializeToString()
with open("testpb",mode="write") as file:
    file.write(serializeToString)




# frame_pb.ParseFromString(serializeToString)

# print("frameID: " + str(frame_pb.id))
# print("Obs")
# for obstacle_pb in frame_pb.obstacle:
#     print(obstacle_pb.id)
#     print(obstacle_pb.pos_x)



