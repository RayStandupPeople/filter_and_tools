#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <thread>

#include "../common/libs/user_struct.h"
#include "../common/libs/decision_struct.h"

#include "../../obstacle_selection/libs/obstacle_selection.h"
#include "../common/libs/display.h"
#include "../common/libs/socket_lib/tcpsocket.h"
#include "../common/libs/socket_lib/data_definition.h"

#include "build/types.pb.h"
#include "build/obstacleSel.pb.h"
#include "build/socket_all.pb.h"

#include "time.h"
#include <signal.h> 

#define OFFSET(st, field)     (size_t)&(((st*)0)->field)
uint32 glo_initsocket_lock;   // global value which for initing socket
bool app_stopped_req = false;     // deal Control C
bool app_shutdown = false;

void sigint_handler(int sig){
	if(sig == SIGINT){
		std::cout << "ctrl+c pressed!" << std::endl;
		app_stopped_req = true;
        // app_shutdown = true;
	}
}


void get_globalpath(laneInfo &globalPath){

    std::ifstream in_file("../../../log/lg_curv.csv",std::ios::in);
    if(!in_file.is_open()){
        std::cout << "ERROR: OPEN  file "<<std::endl;
    }
    std::string line;
    laneNode _node;
    double x;
  	double y;
  	double Heading;
    double d_x;
    double d_y;
 
    std::vector<double> x_array;
    std::vector<double> y_array;
    std::vector<double> Heading_array;

    while(!in_file.eof()){
        getline(in_file, line);
        std::istringstream iss(line);
        iss >> x;
        iss >> y;
        iss >> Heading;
        iss >> d_x;
        iss >> d_y;
        // x_array.push_back(x);
        // y_array.push_back(y);
        // s_array.push_back(s);
        _node.x = x;
        _node.y = y;
        _node.heading = Heading;
        // _node.S = s;
        globalPath.laneNodeInfos.push_back(_node);

    }
    globalPath.nodeNum = globalPath.laneNodeInfos.size();
}

void get_localpath(const Dt_RECORD_LocalizationResult &loc, const laneInfo &glopath, laneInfo &locpath)
{
    int loc_idx =0;
    for(int i=0; i< glopath.laneNodeInfos.size(); ++i)
    {
        if(loc.x == glopath.laneNodeInfos[i].x && loc.y == glopath.laneNodeInfos[i].y)
        loc_idx = i;
    }

    for(int i = loc_idx - 10; i< loc_idx +20; ++i)
    {
        if(i < 0) continue;
        if(i >= glopath.laneNodeInfos.size()) continue;

         locpath.laneNodeInfos.push_back(glopath.laneNodeInfos[i]);
    }

}

std::vector<std::vector<Obj_sel>> get_obstalce_lists(){
    std::ifstream in_file("../../../log/wholeFile_info_pb",std::ios::in);
    if(!in_file.is_open()){
        std::cout << "ERROR: OPEN  file "<<std::endl;
    }
    std::stringstream  CodeStrstream;
    pb_types::LogFile logfile;
    CodeStrstream << in_file.rdbuf();
    in_file.close();
    logfile.ParseFromString(CodeStrstream.str());
    std::vector<std::vector<Obj_sel>>  _obstacle_list_vec;
    std::vector<Obj_sel> _obstacle_list;
    Obj_sel _obstacle_sel;

    for(int i=0;i<logfile.frame_num(); ++i)
    {
        _obstacle_list.clear();
        for(int j=0;j<logfile.frame(i).obstacle_size();++j)
        {
                pb_types::Obstacle _obstacle = logfile.frame(i).obstacle(j);
                _obstacle_sel.id      = _obstacle.id();
                _obstacle_sel.type    = _obstacle.type();
                _obstacle_sel.pos_x   = _obstacle.pos_x();
                _obstacle_sel.pos_y   = _obstacle.pos_y();
                _obstacle_list.push_back(_obstacle_sel);
        }
        _obstacle_list_vec.push_back(_obstacle_list);
    }

    // std::cout << _obstacle_list_vec.size() << std::endl;
    // std::cout << _obstacle_list_vec[1000].size() << std::endl;
    return _obstacle_list_vec;
}

void get_hdMapTrajectory(hdMapTrajectory &_trajectory, const laneInfo &globalpath){

    _trajectory.pathLane[0].segNum = 1;
    _trajectory.pathLane[0].hdmapPathInfo[0].laneNum = 1; //no nearby lane
    _trajectory.pathLane[0].hdmapPathInfo[0].laneInfos[0].nodeNum = globalpath.nodeNum;
    // std::cout<<"_trajectory.pathLane[0].hdmapPathInfo[0].laneInfos[0].nodeNum:" <<globalpath.data.size();
    for(int node_idx=0; node_idx < globalpath.laneNodeInfos.size(); ++node_idx)
    {
        laneNode _lane_node;
        memset(&_lane_node, 0, sizeof(laneNode));
        _lane_node.x = globalpath.laneNodeInfos[node_idx].x;
        _lane_node.y = globalpath.laneNodeInfos[node_idx].y;
        _lane_node.heading = globalpath.laneNodeInfos[node_idx].heading;
        _trajectory.pathLane[0].hdmapPathInfo[0].laneInfos[0].laneNodeInfos.push_back(_lane_node);
    }
}

void get_Dt_RECORD_LocalizationInfo( Dt_RECORD_LocalizationInfo &location, int i)
{
  
    location.LocalizationResult.x = -20 - i;
    location.LocalizationResult.y = 8;
    location.yaw = 10;
}

void get_Dt_RECORD_EnvModelInfos(Dt_RECORD_EnvModelInfos &_envmodle_info)
{
    int i=501;
    std::vector<std::vector<Obj_sel>> obj_list_vet = get_obstalce_lists();
    std::vector<Obj_sel> obj_list = obj_list_vet[i];

    _envmodle_info.obstacle_num = obj_list.size();
    for(int idx =0; idx <obj_list.size(); ++idx)
    {
        Dt_RECORD_Obstacles _obstacle;
        memset(&_obstacle, 0, sizeof(Dt_RECORD_Obstacles));
        _obstacle.id          = obj_list[idx].id;
        _obstacle.type        = obj_list[idx].type;
        _obstacle.pos_x       = obj_list[idx].pos_x;
        _obstacle.pos_y       = obj_list[idx].pos_y;
        _obstacle.heading     = obj_list[idx].heading;
        _obstacle.rel_speed_x = obj_list[idx].rel_speed_x;
        _obstacle.rel_speed_y = obj_list[idx].rel_speed_y;
        _obstacle.abs_speed_x = obj_list[idx].abs_speed_x;
        _obstacle.abs_speed_y = obj_list[idx].abs_speed_y;
        // _obstacle.pos_s       = obj_list[idx].pos_s;
        // _obstacle.pos_d       = obj_list[idx].pos_d;
        _envmodle_info.Obstacles[idx] = _obstacle;
    }
   
}

void get_Dt_RECORD_HdmapFrontPLane(Dt_RECORD_HdmapFrontPLane &_globePLane)
{
    getchar();

    _globePLane.plan_seg_count = 2;
    _globePLane.PlanSeg[0].Lane[0].node_count = 30;
    _globePLane.PlanSeg[1].Lane[0].node_count = 30;

    for(int idx_seg =0; idx_seg <_globePLane.plan_seg_count; ++idx_seg)
    for(int idx_node =0; idx_node <_globePLane.PlanSeg[idx_seg].Lane[0].node_count; ++idx_node)
    {
        _globePLane.PlanSeg[idx_seg].Lane[0].LaneNode[idx_node].hdmap_x = idx_seg *10 + idx_node;
        _globePLane.PlanSeg[idx_seg].Lane[0].LaneNode[idx_node].hdmap_y = idx_seg *5 + idx_node;
    }

}

void receive_zu2Andparse_socket(HdmapToPc_data &rev_hdmapToPC_data, TCPClient &sclient_zu2){

    //发送指令
    size_t ret;
    char command[8];
    memset(command, 0, sizeof(command));
    command[0] = (char)0x01;
    command[1] = (char)0xfe;
    //bool flag = command_send(sclient_tc397, command, sizeof(command));
    bool flag = command_send(sclient_zu2, command, sizeof(command));
    cout << "flag = " << flag << endl;

    if (flag)
    {
        //接收数据
        // HdmapToPc_data rev_data;
        unsigned int id;
        unsigned long long timestamp;
        socket_data_deserialization(sclient_zu2, rev_hdmapToPC_data, id, timestamp);
    }
    else
    {
        sclient_zu2.close();
        sclient_zu2.reconnect(8001, "192.168.1.60");
    }

    if(app_shutdown ==true)
        return ;
 
}
void receive_zu5Andparse_socket(DecisionToPC &rev_DecisionToPC_data, TCPClient &sclient_zu5){
   
    //发送指令
    size_t ret;
    char command[8];
    memset(command, 0, sizeof(command));
    command[0] = (char)0x01;
    command[1] = (char)0xfe;
    //bool flag = command_send(sclient_tc397, command, sizeof(command));
    bool flag = command_send(sclient_zu5, command, sizeof(command));
    cout << "flag = " << flag << endl;

    if (flag)
    {
        //接收数据
        // HdmapToPc_data rev_data;
        unsigned int id;
        unsigned long long timestamp;
        std::cout<<"from" <<std::endl;
        socket_data_deserialization(sclient_zu5, rev_DecisionToPC_data, id, timestamp);
        std::cout << "end" <<std::endl;
    }
    else
    {
        sclient_zu5.close();
        sclient_zu5.reconnect(8001, "192.168.1.70");
    }

    if(app_shutdown ==true)
        return;
}
void log_protobuf_file( DecisionToPC &rev_DecisionToPC_data, uint32 index, socket_all::LogFile &logfile, const std::string &file_name)
{

        logfile.set_frame_total_num(index);
        socket_all::Frame *frame_pb = logfile.add_frame();
        frame_pb->set_frame_id(index);

        /// DecisionToPC
        socket_all::DecisionToPC  *DecisionToPC_pb = new socket_all::DecisionToPC();

        // Dt_RECORD_HdmapInfo
        socket_all::Dt_RECORD_HdmapInfo *Dt_RECORD_HdmapInfo_pb = new socket_all::Dt_RECORD_HdmapInfo();
        Dt_RECORD_HdmapInfo_pb->set_planpath(rev_DecisionToPC_data.my_hdmapInfo.planpath);
        Dt_RECORD_HdmapInfo_pb->set_origin_x(rev_DecisionToPC_data.my_hdmapInfo.origin_x);
        Dt_RECORD_HdmapInfo_pb->set_origin_y(rev_DecisionToPC_data.my_hdmapInfo.origin_y);
        Dt_RECORD_HdmapInfo_pb->set_origin_z(rev_DecisionToPC_data.my_hdmapInfo.origin_z);
        Dt_RECORD_HdmapInfo_pb->set_origin_yaw(rev_DecisionToPC_data.my_hdmapInfo.origin_yaw);
        Dt_RECORD_HdmapInfo_pb->set_goal_x(rev_DecisionToPC_data.my_hdmapInfo.goal_x);
        Dt_RECORD_HdmapInfo_pb->set_goal_y(rev_DecisionToPC_data.my_hdmapInfo.goal_y);
        Dt_RECORD_HdmapInfo_pb->set_goal_z(rev_DecisionToPC_data.my_hdmapInfo.goal_z);
        Dt_RECORD_HdmapInfo_pb->set_goal_yaw(rev_DecisionToPC_data.my_hdmapInfo.goal_yaw);
        DecisionToPC_pb->set_allocated_my_hdmapinfo(Dt_RECORD_HdmapInfo_pb);
        
        // // Dt_RECORD_HdmapFrontPLane
        socket_all::Dt_RECORD_HdmapFrontPLane *Dt_RECORD_HdmapFrontPLane_pb = new socket_all::Dt_RECORD_HdmapFrontPLane();
        Dt_RECORD_HdmapFrontPLane_pb->set_plan_seg_count(rev_DecisionToPC_data.my_hdmapFrontPLaneInfo.plan_seg_count);
        for(uint32 seg =0; seg < rev_DecisionToPC_data.my_hdmapFrontPLaneInfo.plan_seg_count; seg++)
        {
            socket_all::Dt_RECORD_PlanSeg *Dt_RECORD_PlanSeg_pb = Dt_RECORD_HdmapFrontPLane_pb->add_planseg();
            Dt_RECORD_PlanSeg_pb->set_lane_count(rev_DecisionToPC_data.my_hdmapFrontPLaneInfo.PlanSeg[seg].lane_count);
            for(uint32 lane=0; lane <rev_DecisionToPC_data.my_hdmapFrontPLaneInfo.PlanSeg[seg].lane_count; ++lane)
            {
                socket_all::Dt_RECORD_HdMapLane *Dt_RECORD_HdMapLane_pb = Dt_RECORD_PlanSeg_pb->add_lane();
                Dt_RECORD_HdMapLane_pb->set_node_count       (rev_DecisionToPC_data.my_hdmapFrontPLaneInfo.PlanSeg[seg].Lane[lane].node_count);
                Dt_RECORD_HdMapLane_pb->set_laneid           (rev_DecisionToPC_data.my_hdmapFrontPLaneInfo.PlanSeg[seg].Lane[lane].laneID);
                Dt_RECORD_HdMapLane_pb->set_change_lane_flag (rev_DecisionToPC_data.my_hdmapFrontPLaneInfo.PlanSeg[seg].Lane[lane].change_lane_flag);
                Dt_RECORD_HdMapLane_pb->set_lane_width       (rev_DecisionToPC_data.my_hdmapFrontPLaneInfo.PlanSeg[seg].Lane[lane].lane_width);
                Dt_RECORD_HdMapLane_pb->set_road_id          (rev_DecisionToPC_data.my_hdmapFrontPLaneInfo.PlanSeg[seg].Lane[lane].road_ID);
                for(unsigned node=0; node< rev_DecisionToPC_data.my_hdmapFrontPLaneInfo.PlanSeg[seg].Lane[lane].node_count; ++node)
                {
                    socket_all::Dt_RECORD_LaneNode *Dt_RECORD_LaneNode_pb = Dt_RECORD_HdMapLane_pb->add_lanenode();
                    Dt_RECORD_LaneNode_pb->set_hdmap_x(rev_DecisionToPC_data.my_hdmapFrontPLaneInfo.PlanSeg[seg].Lane[lane].LaneNode[node].hdmap_x);
                    Dt_RECORD_LaneNode_pb->set_hdmap_y(rev_DecisionToPC_data.my_hdmapFrontPLaneInfo.PlanSeg[seg].Lane[lane].LaneNode[node].hdmap_y);
                    Dt_RECORD_LaneNode_pb->set_hdmap_z(rev_DecisionToPC_data.my_hdmapFrontPLaneInfo.PlanSeg[seg].Lane[lane].LaneNode[node].hdmap_z);
                    Dt_RECORD_LaneNode_pb->set_heading(rev_DecisionToPC_data.my_hdmapFrontPLaneInfo.PlanSeg[seg].Lane[lane].LaneNode[node].heading);

                }
            }
        }
        DecisionToPC_pb->set_allocated_my_hdmapfrontplaneinfo(Dt_RECORD_HdmapFrontPLane_pb);

        // Dt_RECORD_LocalizationInfo
        socket_all::Dt_RECORD_LocalizationInfo *Dt_RECORD_LocalizationInfo_pb = new socket_all::Dt_RECORD_LocalizationInfo();
        Dt_RECORD_LocalizationInfo_pb->set_time_stamp(rev_DecisionToPC_data.my_localizationInfo.time_stamp);
        Dt_RECORD_LocalizationInfo_pb->set_latitude(rev_DecisionToPC_data.my_localizationInfo.Latitude);
        Dt_RECORD_LocalizationInfo_pb->set_longitude(rev_DecisionToPC_data.my_localizationInfo.Longitude);
        Dt_RECORD_LocalizationInfo_pb->set_yaw(rev_DecisionToPC_data.my_localizationInfo.yaw);
        Dt_RECORD_LocalizationInfo_pb->set_yawrate(rev_DecisionToPC_data.my_localizationInfo.yawrate);
        socket_all::Dt_RECORD_LocalizationResult *Dt_RECORD_LocalizationResult_pb = new socket_all::Dt_RECORD_LocalizationResult();
        Dt_RECORD_LocalizationResult_pb->set_valid(rev_DecisionToPC_data.my_localizationInfo.LocalizationResult.valid);
        Dt_RECORD_LocalizationResult_pb->set_x(rev_DecisionToPC_data.my_localizationInfo.LocalizationResult.x);
        Dt_RECORD_LocalizationResult_pb->set_y(rev_DecisionToPC_data.my_localizationInfo.LocalizationResult.y);
        Dt_RECORD_LocalizationResult_pb->set_z(rev_DecisionToPC_data.my_localizationInfo.LocalizationResult.z);
        Dt_RECORD_LocalizationInfo_pb->set_allocated_localizationresult(Dt_RECORD_LocalizationResult_pb);
        DecisionToPC_pb->set_allocated_my_localizationinfo(Dt_RECORD_LocalizationInfo_pb);
         
         // Dt_RECORD_TrajectoryPointsInfos
        socket_all::Dt_RECORD_TrajectoryPointsInfos *Dt_RECORD_TrajectoryPointsInfos_pb = new socket_all::Dt_RECORD_TrajectoryPointsInfos();
        Dt_RECORD_TrajectoryPointsInfos_pb->set_origin_yaw(rev_DecisionToPC_data.my_trajectoryPointsInfo.origin_yaw);
        Dt_RECORD_TrajectoryPointsInfos_pb->set_point_num(rev_DecisionToPC_data.my_trajectoryPointsInfo.point_num);
        Dt_RECORD_TrajectoryPointsInfos_pb->set_array_length_dummy_0(rev_DecisionToPC_data.my_trajectoryPointsInfo.Array_Length_Dummy_0);
        Dt_RECORD_TrajectoryPointsInfos_pb->set_decision(rev_DecisionToPC_data.my_trajectoryPointsInfo.decision);
        Dt_RECORD_TrajectoryPointsInfos_pb->set_hold(rev_DecisionToPC_data.my_trajectoryPointsInfo.hold);
        Dt_RECORD_TrajectoryPointsInfos_pb->set_direction(rev_DecisionToPC_data.my_trajectoryPointsInfo.direction);
        for(uint32 point=0; point < rev_DecisionToPC_data.my_trajectoryPointsInfo.point_num;++point)
        {
            socket_all::Dt_RECORD_TrajectoryPoints *Dt_RECORD_TrajectoryPoints_pb = Dt_RECORD_TrajectoryPointsInfos_pb->add_trajectorypoints();
            Dt_RECORD_TrajectoryPoints_pb->set_x(rev_DecisionToPC_data.my_trajectoryPointsInfo.TrajectoryPoints[point].x);
            Dt_RECORD_TrajectoryPoints_pb->set_y(rev_DecisionToPC_data.my_trajectoryPointsInfo.TrajectoryPoints[point].y);
            Dt_RECORD_TrajectoryPoints_pb->set_theta(rev_DecisionToPC_data.my_trajectoryPointsInfo.TrajectoryPoints[point].theta);
            Dt_RECORD_TrajectoryPoints_pb->set_kappa(rev_DecisionToPC_data.my_trajectoryPointsInfo.TrajectoryPoints[point].kappa);
            Dt_RECORD_TrajectoryPoints_pb->set_v(rev_DecisionToPC_data.my_trajectoryPointsInfo.TrajectoryPoints[point].v);
            Dt_RECORD_TrajectoryPoints_pb->set_lane_no(rev_DecisionToPC_data.my_trajectoryPointsInfo.TrajectoryPoints[point].lane_no);
            Dt_RECORD_TrajectoryPoints_pb->set_t(rev_DecisionToPC_data.my_trajectoryPointsInfo.TrajectoryPoints[point].t);
        }
        DecisionToPC_pb->set_allocated_my_trajectorypointsinfo(Dt_RECORD_TrajectoryPointsInfos_pb);

         // Dt_RECORD_AccInfo
        socket_all::Dt_RECORD_AccInfo *Dt_RECORD_AccInfo_pb = new socket_all::Dt_RECORD_AccInfo();
        Dt_RECORD_AccInfo_pb->set_esp_vehspd(rev_DecisionToPC_data.my_vehicleInfo.ESP_VehSpd);
        Dt_RECORD_AccInfo_pb->set_yrs_yawrate(rev_DecisionToPC_data.my_vehicleInfo.YRS_YawRate);
        DecisionToPC_pb->set_allocated_my_vehicleinfo(Dt_RECORD_AccInfo_pb);

         // Dt_RECORD_EnvModelInfos
        socket_all::Dt_RECORD_EnvModelInfos *Dt_RECORD_EnvModelInfos_pb = new socket_all::Dt_RECORD_EnvModelInfos();
        Dt_RECORD_EnvModelInfos_pb->set_frame_index(rev_DecisionToPC_data.my_envModelInfo.frame_index);
        Dt_RECORD_EnvModelInfos_pb->set_obstacle_num(rev_DecisionToPC_data.my_envModelInfo.obstacle_num);
        for(uint32 obstacle=0; obstacle < rev_DecisionToPC_data.my_envModelInfo.obstacle_num; ++obstacle)
        {
            socket_all::Dt_RECORD_Obstacles *Dt_RECORD_Obstacles_pb = Dt_RECORD_EnvModelInfos_pb->add_obstacles();
            Dt_RECORD_Obstacles_pb->set_id(rev_DecisionToPC_data.my_envModelInfo.Obstacles[obstacle].id);
            Dt_RECORD_Obstacles_pb->set_type(rev_DecisionToPC_data.my_envModelInfo.Obstacles[obstacle].type);
            Dt_RECORD_Obstacles_pb->set_confidence_state(rev_DecisionToPC_data.my_envModelInfo.Obstacles[obstacle].confidence_state);
            Dt_RECORD_Obstacles_pb->set_state(rev_DecisionToPC_data.my_envModelInfo.Obstacles[obstacle].state);
            Dt_RECORD_Obstacles_pb->set_cipv_flag(rev_DecisionToPC_data.my_envModelInfo.Obstacles[obstacle].CIPV_flag);
            Dt_RECORD_Obstacles_pb->set_pos_x(rev_DecisionToPC_data.my_envModelInfo.Obstacles[obstacle].pos_x);
            Dt_RECORD_Obstacles_pb->set_pos_y(rev_DecisionToPC_data.my_envModelInfo.Obstacles[obstacle].pos_y);
            Dt_RECORD_Obstacles_pb->set_rel_acc_x(rev_DecisionToPC_data.my_envModelInfo.Obstacles[obstacle].rel_acc_x);
            Dt_RECORD_Obstacles_pb->set_rel_acc_y(rev_DecisionToPC_data.my_envModelInfo.Obstacles[obstacle].rel_acc_y);
            Dt_RECORD_Obstacles_pb->set_abs_speed_x(rev_DecisionToPC_data.my_envModelInfo.Obstacles[obstacle].abs_speed_x);
            Dt_RECORD_Obstacles_pb->set_abs_speed_y(rev_DecisionToPC_data.my_envModelInfo.Obstacles[obstacle].abs_speed_y);
            Dt_RECORD_Obstacles_pb->set_abs_acc_x(rev_DecisionToPC_data.my_envModelInfo.Obstacles[obstacle].abs_acc_x);
            Dt_RECORD_Obstacles_pb->set_abs_acc_y(rev_DecisionToPC_data.my_envModelInfo.Obstacles[obstacle].abs_acc_y);
            Dt_RECORD_Obstacles_pb->set_heading(rev_DecisionToPC_data.my_envModelInfo.Obstacles[obstacle].heading);
            Dt_RECORD_Obstacles_pb->set_length(rev_DecisionToPC_data.my_envModelInfo.Obstacles[obstacle].length);
            Dt_RECORD_Obstacles_pb->set_width(rev_DecisionToPC_data.my_envModelInfo.Obstacles[obstacle].width);
            Dt_RECORD_Obstacles_pb->set_height(rev_DecisionToPC_data.my_envModelInfo.Obstacles[obstacle].height);
        }
        Dt_RECORD_EnvModelInfos_pb->set_traffic_sign_num(rev_DecisionToPC_data.my_envModelInfo.traffic_sign_num);
       
        for(uint32 sign=0; sign < rev_DecisionToPC_data.my_envModelInfo.traffic_sign_num;++sign)
        {
            socket_all::Dt_RECORD_TrafficSigns *Dt_RECORD_TrafficSigns_pb = Dt_RECORD_EnvModelInfos_pb->add_trafficsigns();
            Dt_RECORD_TrafficSigns_pb->set_id(rev_DecisionToPC_data.my_envModelInfo.TrafficSigns[sign].id);
            Dt_RECORD_TrafficSigns_pb->set_type(rev_DecisionToPC_data.my_envModelInfo.TrafficSigns[sign].type);
            Dt_RECORD_TrafficSigns_pb->set_confidence(rev_DecisionToPC_data.my_envModelInfo.TrafficSigns[sign].confidence);
            Dt_RECORD_TrafficSigns_pb->set_pos_x(rev_DecisionToPC_data.my_envModelInfo.TrafficSigns[sign].pos_x);
            Dt_RECORD_TrafficSigns_pb->set_pos_y(rev_DecisionToPC_data.my_envModelInfo.TrafficSigns[sign].pos_y);
            Dt_RECORD_TrafficSigns_pb->set_pos_z(rev_DecisionToPC_data.my_envModelInfo.TrafficSigns[sign].pos_z);
        }
        
        // for(uint32 grid_a =0; grid_a < 400; ++grid_a)
        // {
        //     socket_all::Dt_ARRAY_200_GridInfoX *Dt_ARRAY_200_GridInfoX_pb = Dt_RECORD_EnvModelInfos_pb->add_obstaclegridmap();
        //     for(uint32 grid_b =0; grid_b < 200; ++grid_b)
        //     {
        //         Dt_ARRAY_200_GridInfoX_pb->add_grid_node(rev_DecisionToPC_data.my_envModelInfo.ObstacleGridMap[grid_a][grid_b]);
        //     }
        // }

        DecisionToPC_pb->set_allocated_my_envmodelinfo(Dt_RECORD_EnvModelInfos_pb);
    frame_pb->set_allocated_decisiontopc(DecisionToPC_pb);  

    if(app_stopped_req == true)
    {
        std::ofstream ofile("../data/"+file_name,std::ios::out);
            if(!ofile.is_open())
                std::cout << "can not find data_convert file" << std::endl;
            logfile.SerializeToOstream(&ofile);
            ofile.close();
        std::cout << "***************LOG FILE SAVE SUCCESSFULLY*************" <<std::endl;
        app_shutdown =true;
    }
    
}
void read_from_proto_file(DecisionToPC &rev_DecisionToPC_data, const std::string &file_name){
    std::ifstream in_file("../data/"+file_name,std::ios::in);
    if(!in_file.is_open()){
        std::cout << "ERROR: OPEN  file "<<std::endl;
    }
    std::stringstream  CodeStrstream;
    socket_all::LogFile logfile;
    CodeStrstream << in_file.rdbuf();
    in_file.close();
    logfile.ParseFromString(CodeStrstream.str());
    std::cout << "Total frame num:" <<logfile.frame_total_num() << std::endl;
    
    socket_all::Frame frame_pb = logfile.frame(0);
    socket_all::DecisionToPC DecisionToPC_pb =frame_pb.decisiontopc();
    // std::cout << "onpayh" <<HdmapToPc_data_pb.g_hdmapinfo().planpath();
    // rev_DecisionToPC_data.my_hdmapInfo.planpath = DecisionToPC_pb.my_hdmapinfo().planpath();
    // for(int i=0;i<logfile.frame_num(); ++i)
    // {
    //     _obstacle_list.clear();
    //     for(int j=0;j<logfile.frame(i).obstacle_size();++j)
    //     {
    //             pb_types::Obstacle _obstacle = logfile.frame(i).obstacle(j);
    //             _obstacle_sel.id      = _obstacle.id();
    //             _obstacle_sel.type    = _obstacle.type();
    //             _obstacle_sel.pos_x   = _obstacle.pos_x();
    //             _obstacle_sel.pos_y   = _obstacle.pos_y();
    //             _obstacle_list.push_back(_obstacle_sel);
    //     }
    //     _obstacle_list_vec.push_back(_obstacle_list);
    // }

    // std::cout << _obstacle_list_vec.size() << std::endl;
    // std::cout << _obstacle_list_vec[1000].size() << std::endl;
    
}
int main(int argc, const char** argv) {
        clock_t g_t_s, g_t_e;
    /* DATA Define and init */
    // sensors data
    hdMapTrajectory Trajectory; //

    Dt_RECORD_TrajectoryPointsInfos trajectoryPointsInfo;
    Dt_RECORD_HdmapInfo globePLanehdmapInfos;
    Dt_RECORD_HdmapFrontPLane globePLane;
    Dt_RECORD_HdmapLocalLane localPLanne;
    Dt_RECORD_LocalizationInfo localInfos;
    Dt_RECORD_EnvModelInfos envModelInfo;
    Dt_RECORD_AccInfo vehicleInfo;	
    EgoConfigPara ego_config;
    objSecList selectObj;

    memset(&Trajectory,0,sizeof(hdMapTrajectory));

    memset(&trajectoryPointsInfo,0,sizeof(Dt_RECORD_TrajectoryPointsInfos));
    memset(&globePLanehdmapInfos,0,sizeof(Dt_RECORD_HdmapInfo));
    memset(&globePLane,0,sizeof(Dt_RECORD_HdmapFrontPLane));
    memset(&localPLanne,0,sizeof(Dt_RECORD_HdmapLocalLane));
    memset(&localInfos,0,sizeof(Dt_RECORD_LocalizationInfo));
    memset(&envModelInfo,0,sizeof(Dt_RECORD_EnvModelInfos));  
    memset(&vehicleInfo,0,sizeof(Dt_RECORD_AccInfo));    
    memset(&selectObj,0,sizeof(objSecList));

    // socket data
    // HdmapToPc_data rev_hdmapToPC_data;
    DecisionToPC   rev_DecisionToPC_data;
    // TCPClient sclient_zu2;
    // TCPClient sclient_zu5;

    // alogorithm data
    decision decision_obj;   

    // init socket and mode
    string MODE;
    if(argc==1) 
    {
        MODE = "pure";
        std::cout << "Mode: pure, just display without log" << std::endl;
    }
    else  
    {
        MODE = static_cast<string>(argv[1]);
        
        if(MODE == "log")
            std::cout << "Mode: log, display and log" << std::endl;

        else if(MODE ==  "replay" && argc==3)
            std::cout << "Mode: replay, display proto file data" << std::endl;

        else if(MODE ==  "replay" && argc==2)
            std::cout << "Mode: replay, display User simulate data" << std::endl;

        else
        {
            std::cout << "ERROR MODE"<<std::endl;
            return -1;
        }
        
        
    }
    
    uint32 frame_index =0; //logfile frame index
    socket_all::LogFile logfile; //logfile obj
       
    // CYCLE STRUCT
    while(true)
    {
        g_t_s = clock();
        /* PARSE: form socket or file*/ 
        if(MODE == "pure" || MODE == "log") // Pure, Log  (ONline)
        {
            // init socket
            if(glo_initsocket_lock ==0)
            {
                std::cout << "connecting socket..." << std::endl;
                // TCPClient sclient_zu2(8001, "192.168.1.60");
                
                // TCPClient sclient_zu5(static_cast<unsigned short int>(8001), "192.168.1.70");

                glo_initsocket_lock =1;
            }
            // read from socket
            // receive_zu5Andparse_socket(rev_DecisionToPC_data, sclient_zu5);

            // thread socket_thread_zu2(receive_zu2Andparse_socket, std::ref(rev_hdmapToPC_data), std::ref(sclient_zu2));
            // thread socket_thread_zu5(receive_zu5Andparse_socket, std::ref(rev_DecisionToPC_data),std::ref(sclient_zu5));
            // socket_thread_zu2.join();
            // socket_thread_zu5.join();
            if(MODE =="log") // Log Proto 
            {
                std::string file_name = static_cast<string>(argv[2]);
                // clock_t t_s = clock();
                    log_protobuf_file(rev_DecisionToPC_data, frame_index, logfile, file_name);
                // clock_t t_e = clock();
                // std::cout << "log time cost: " << double(t_e -t_s)/(clock_t(1)) << std::endl;;
            }
            // app switch
            if(app_stopped_req == true)
                app_shutdown =true;

        }
        if(MODE == "replay") // Replay  (OFF line)
        {
            // read from proto
            // assign value
            if(argc ==3)
            {
                std::string file_name = static_cast<string>(argv[2]);
                read_from_proto_file( rev_DecisionToPC_data, file_name);
                if(app_stopped_req == true)
                    app_shutdown =true;   
            }
            else
            {
                // read_from_simulate_data();
                if(app_stopped_req == true)
                    app_shutdown =true;   
            }
            
            if(app_stopped_req == true)
                app_shutdown =true;   
                 
        }

        /*    ASSIGN VALUES    */
        // laneInfo globalpath;
        // get_globalpath(globalpath);
        // get_hdMapTrajectory(Trajectory, globalpath);
        // Trajectory = rev_DecisionToPC_data.my_trajectoryPointsInfo;
        trajectoryPointsInfo = rev_DecisionToPC_data.my_trajectoryPointsInfo; // out2 next SWC
        globePLanehdmapInfos = rev_DecisionToPC_data.my_hdmapInfo;
        globePLane = rev_DecisionToPC_data.my_hdmapFrontPLaneInfo;
        localPLanne = rev_DecisionToPC_data.my_hdmapLocalLaneInfo;
        localInfos = rev_DecisionToPC_data.my_localizationInfo;
        envModelInfo = rev_DecisionToPC_data.my_envModelInfo;
        vehicleInfo = rev_DecisionToPC_data.my_vehicleInfo;
        // get_Dt_RECORD_EnvModelInfos(envModelInfo);

        std::cout << "current frame idx:" << frame_index <<std::endl;

        /*   ALGORITHMs  */
        decision_obj.ObjDetect(1, &Trajectory, &globePLanehdmapInfos, &globePLane, &localPLanne, &localInfos, &envModelInfo, ego_config, &selectObj);
        

        /*   DISPALY    */
        plot(trajectoryPointsInfo, globePLanehdmapInfos, globePLane, localInfos, envModelInfo,vehicleInfo, selectObj);
        
        /* OTHER ITEMS */
        signal(SIGINT, sigint_handler);
        frame_index++;
        g_t_e = clock();
        std::cout << "----> PER frame time cost: " << (double)(g_t_e -g_t_s)/CLOCKS_PER_SEC << "s"<< std::endl;;
        if(app_shutdown ==true)
            return 0;
    }
    
    return 0;
}