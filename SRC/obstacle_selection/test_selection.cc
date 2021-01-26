#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <thread>

#include <stdio.h>
#include <string.h>	 
#include <stdlib.h>	
#include <unistd.h>	
#include <arpa/inet.h>
#include <sys/socket.h>

#include "../common/libs/user_struct.h"
#include "../common/libs/decision_struct.h"

#include "../../obstacle_selection/libs/obstacle_selection.h"
#include "../common/libs/display.h"
#include "../common/libs/socket_lib/tcpsocket.h"
#include "../common/libs/socket_lib/data_definition.h"
#include "../../obstacle_selection/libs/static_objSel.h"
#include "../../obstacle_selection/libs/objSelectFusion.h"

#include "build/types.pb.h"
#include "build/obstacleSel.pb.h"
#include "build/socket_all.pb.h"

#include "time.h"
#include <signal.h> 
// #define SOCKETON


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

    std::ifstream in_file("../../../log/gwh_lc_bug.csv",std::ios::in);
    // std::ifstream in_file("../../../log/trunning_termi_bug.csv",std::ios::in);

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



    _trajectory.pathLane[0].segNum = 3;
    
    int begin_ =0;
    int end_ =0;
    for(int node_idx=0; node_idx < globalpath.laneNodeInfos.size()*5/10; ++node_idx)
    {
        laneNode _lane_node;
        memset(&_lane_node, 0, sizeof(laneNode));
        _lane_node.x = globalpath.laneNodeInfos[node_idx].x;
        _lane_node.y = globalpath.laneNodeInfos[node_idx].y;
        _lane_node.heading = globalpath.laneNodeInfos[node_idx].heading;
        _trajectory.pathLane[0].hdmapPathInfo[0].laneInfos[0].laneNodeInfos.push_back(_lane_node);
        end_ = node_idx;
    }
    // std::cout << "statt :  end" << begin_ << "   " << end_ << std::endl;
    // std::cout << "begin :  end" << _trajectory.pathLane[0].hdmapPathInfo[0].laneInfos[0].laneNodeInfos.front().x << \
    " "<<  _trajectory.pathLane[0].hdmapPathInfo[0].laneInfos[0].laneNodeInfos.back().x << std::endl;

    _trajectory.pathLane[0].hdmapPathInfo[0].laneNum = 1; //no nearby lane
    _trajectory.pathLane[0].hdmapPathInfo[0].laneInfos[0].nodeNum = end_ - begin_;


    begin_=end_;
    for(int node_idx=0; node_idx < globalpath.laneNodeInfos.size()*1/10; ++node_idx)
    {
        laneNode _lane_node;
        memset(&_lane_node, 0, sizeof(laneNode));
        _lane_node.x = globalpath.laneNodeInfos[begin_+ node_idx].x;
        _lane_node.y = globalpath.laneNodeInfos[begin_+ node_idx].y;
        _lane_node.heading = globalpath.laneNodeInfos[begin_ +node_idx].heading;
        _trajectory.pathLane[0].hdmapPathInfo[1].laneInfos[0].laneNodeInfos.push_back(_lane_node);
        end_ = node_idx + begin_;
    }
        // std::cout << "statt : end" << begin_ << " " << end_  << std::endl;
        //  std::cout << "begin :  end" << _trajectory.pathLane[0].hdmapPathInfo[1].laneInfos[0].laneNodeInfos.front().x << \
    " "<<  _trajectory.pathLane[0].hdmapPathInfo[1].laneInfos[0].laneNodeInfos.back().x << std::endl;
     _trajectory.pathLane[0].hdmapPathInfo[1].laneNum = 4; //no nearby lane
    _trajectory.pathLane[0].hdmapPathInfo[1].laneInfos[0].nodeNum = end_ - begin_;


    begin_=end_;
    for(int node_idx=0; node_idx < globalpath.laneNodeInfos.size() *4/10; ++node_idx)
    {
        laneNode _lane_node;
        memset(&_lane_node, 0, sizeof(laneNode));
        _lane_node.x = globalpath.laneNodeInfos[begin_+ node_idx].x;
        _lane_node.y = globalpath.laneNodeInfos[begin_+ node_idx].y;
        _lane_node.heading = globalpath.laneNodeInfos[begin_ + node_idx].heading;
        _trajectory.pathLane[0].hdmapPathInfo[2].laneInfos[0].laneNodeInfos.push_back(_lane_node);
        end_ = node_idx + begin_;
    }
    // std::cout << "statt : end" << begin_ << " " << end_ <<  std::endl;
    //  std::cout << "begin :  end" << _trajectory.pathLane[0].hdmapPathInfo[2].laneInfos[0].laneNodeInfos.front().x << \
    // " "<<  _trajectory.pathLane[0].hdmapPathInfo[2].laneInfos[0].laneNodeInfos.back().x << std::endl;
    _trajectory.pathLane[0].hdmapPathInfo[2].laneNum = 1; //no nearby lane
    _trajectory.pathLane[0].hdmapPathInfo[2].laneInfos[0].nodeNum = end_ - begin_;

}

void get_Dt_RECORD_LocalizationInfo( Dt_RECORD_LocalizationInfo &location)
{
  
    location.LocalizationResult.x = 85.1;
    location.LocalizationResult.y = 106.2;
    // location.yaw = 95.29;
    location.yaw = 216.2;

    
}

void get_Dt_RECORD_EnvModelInfos(Dt_RECORD_EnvModelInfos &_envmodle_info)
{
    int i=501;
    std::vector<std::vector<Obj_sel>> obj_list_vet = get_obstalce_lists();
    std::vector<Obj_sel> obj_list = obj_list_vet[i];

    // _envmodle_info.obstacle_num = obj_list.size();
    // for(int idx =0; idx <obj_list.size(); ++idx)
    // {
    //     Dt_RECORD_Obstacles _obstacle;
    //     memset(&_obstacle, 0, sizeof(Dt_RECORD_Obstacles));
    //     _obstacle.id          = obj_list[idx].id;
    //     _obstacle.type        = static_cast<uint8>(obj_list[idx].type);
    //     _obstacle.pos_x       = obj_list[idx].pos_x;
    //     _obstacle.pos_y       = obj_list[idx].pos_y;
    //     _obstacle.heading     = obj_list[idx].heading;
    //     _obstacle.rel_speed_x = obj_list[idx].rel_speed_x;
    //     _obstacle.rel_speed_y = obj_list[idx].rel_speed_y;
    //     _obstacle.abs_speed_x = obj_list[idx].abs_speed_x;
    //     _obstacle.abs_speed_y = obj_list[idx].abs_speed_y;
    //     // _obstacle.pos_s       = obj_list[idx].pos_s;
    //     // _obstacle.pos_d       = obj_list[idx].pos_d;
    //     _envmodle_info.Obstacles[idx] = _obstacle;

    // }

      _envmodle_info.obstacle_num =5;
        Dt_RECORD_Obstacles _obstacle;

        memset(&_obstacle, 0, sizeof(Dt_RECORD_Obstacles));
        _obstacle.id          = 145;
        _obstacle.type        = 1;
        _obstacle.pos_x       = -4.1;
        _obstacle.pos_y       = 3.92;
        _obstacle.heading     = 0;
        _obstacle.rel_speed_x = -1.3;
        _obstacle.rel_speed_y = 0;
        _obstacle.abs_speed_x = 0;
        _obstacle.abs_speed_y = 0;
        _envmodle_info.Obstacles[0] = _obstacle;

        memset(&_obstacle, 0, sizeof(Dt_RECORD_Obstacles));
        _obstacle.id          = 150;
        _obstacle.type        = 1;
        _obstacle.pos_x       = -15;
        _obstacle.pos_y       = 8;
        _obstacle.heading     = 0;
        _obstacle.rel_speed_x = 0;
        _obstacle.rel_speed_y = 0;
        _obstacle.abs_speed_x = 0;
        _obstacle.abs_speed_y = 0;
        // _obstacle.pos_s       = obj_list[idx].pos_s;
        // _obstacle.pos_d       = obj_list[idx].pos_d;
        _envmodle_info.Obstacles[1] = _obstacle;

        memset(&_obstacle, 0, sizeof(Dt_RECORD_Obstacles));
        _obstacle.id          = 213;
        _obstacle.type        = 1;
        _obstacle.pos_x       = 4.2;
        _obstacle.pos_y       = 3.8;
        _obstacle.heading     = 0;
        _obstacle.rel_speed_x = 0;
        _obstacle.rel_speed_y = 0;
        _obstacle.abs_speed_x = 0;
        _obstacle.abs_speed_y = 0;
        // _obstacle.pos_s       = obj_list[idx].pos_s;
        // _obstacle.pos_d       = obj_list[idx].pos_d;
        _envmodle_info.Obstacles[2] = _obstacle;

        memset(&_obstacle, 0, sizeof(Dt_RECORD_Obstacles));
        _obstacle.id          = 19;
        _obstacle.type        = 0;
        _obstacle.pos_x       = 3.6;
        _obstacle.pos_y       = -3.6;
        _obstacle.heading     = 0;
        _obstacle.rel_speed_x = 0;
        _obstacle.rel_speed_y = 0;
        _obstacle.abs_speed_x = 0;
        _obstacle.abs_speed_y = 0;
        // _obstacle.pos_s       = obj_list[idx].pos_s;
        // _obstacle.pos_d       = obj_list[idx].pos_d;
        _envmodle_info.Obstacles[3] = _obstacle;

        memset(&_obstacle, 0, sizeof(Dt_RECORD_Obstacles));
        _obstacle.id          = 20;
        _obstacle.type        = 1;
        _obstacle.pos_x       = 5.2;
        _obstacle.pos_y       = 3.4;
        _obstacle.heading     = 0;
        _obstacle.rel_speed_x = 0;
        _obstacle.rel_speed_y = 0;
        _obstacle.abs_speed_x = 0;
        _obstacle.abs_speed_y = 0;
        // _obstacle.pos_s       = obj_list[idx].pos_s;
        // _obstacle.pos_d       = obj_list[idx].pos_d;
        _envmodle_info.Obstacles[4] = _obstacle;


        // Grid Map
        // _envmodle_info.ObstacleGridMap[399][99] =1; // 后轴中心



<<<<<<< HEAD
        for(uint32 x=360-1; x<400; ++x)// front 10 ~ 20 meter
        for(uint32 y=100-12;  y<100+12; ++y) // vehicle_mid = 100 
        {
             _envmodle_info.ObstacleGridMap[x][y]=1;
        }

        for(uint32 x=380-1; x<420; ++x)// front 10 ~ 20 meter
        for(uint32 y=100+20;  y<100+42; ++y) // vehicle_mid = 100 
        {
             _envmodle_info.ObstacleGridMap[x][y]=1;
        }
=======
        // for(uint32 x=390-1; x<400; ++x)// front 10 ~ 20 meter
        // for(uint32 y=122-1;  y<136; ++y) // vehicle_mid = 100 
        // {
        //      _envmodle_info.ObstacleGridMap[x][y]=1;
        // }
>>>>>>> feature/socketfromWin2021-0119


_envmodle_info.ObstacleGridMap[149][167] = 1;
_envmodle_info.ObstacleGridMap[149][167] = 1;
_envmodle_info.ObstacleGridMap[150][167] = 1;
_envmodle_info.ObstacleGridMap[149][168] = 1;
_envmodle_info.ObstacleGridMap[150][168] = 1;
_envmodle_info.ObstacleGridMap[148][159] = 1;
_envmodle_info.ObstacleGridMap[149][159] = 1;
_envmodle_info.ObstacleGridMap[148][160] = 1;
_envmodle_info.ObstacleGridMap[149][160] = 1;
_envmodle_info.ObstacleGridMap[149][161] = 1;
_envmodle_info.ObstacleGridMap[150][161] = 1;
_envmodle_info.ObstacleGridMap[149][162] = 1;
_envmodle_info.ObstacleGridMap[150][162] = 1;
_envmodle_info.ObstacleGridMap[149][163] = 1;
_envmodle_info.ObstacleGridMap[150][163] = 1;
_envmodle_info.ObstacleGridMap[149][164] = 1;
_envmodle_info.ObstacleGridMap[150][164] = 1;
_envmodle_info.ObstacleGridMap[150][165] = 1;
_envmodle_info.ObstacleGridMap[150][166] = 1;
_envmodle_info.ObstacleGridMap[150][159] = 1;
_envmodle_info.ObstacleGridMap[150][160] = 1;
_envmodle_info.ObstacleGridMap[147][168] = 1;
_envmodle_info.ObstacleGridMap[148][168] = 1;
_envmodle_info.ObstacleGridMap[147][169] = 1;
_envmodle_info.ObstacleGridMap[148][169] = 1;
_envmodle_info.ObstacleGridMap[147][170] = 1;
_envmodle_info.ObstacleGridMap[148][170] = 1;
_envmodle_info.ObstacleGridMap[147][171] = 1;
_envmodle_info.ObstacleGridMap[148][171] = 1;
_envmodle_info.ObstacleGridMap[148][171] = 1;
_envmodle_info.ObstacleGridMap[149][171] = 1;
_envmodle_info.ObstacleGridMap[148][172] = 1;
_envmodle_info.ObstacleGridMap[149][172] = 1;
_envmodle_info.ObstacleGridMap[149][173] = 1;
_envmodle_info.ObstacleGridMap[150][173] = 1;
_envmodle_info.ObstacleGridMap[149][167] = 1;
_envmodle_info.ObstacleGridMap[150][167] = 1;
_envmodle_info.ObstacleGridMap[149][168] = 1;
_envmodle_info.ObstacleGridMap[150][168] = 1;
_envmodle_info.ObstacleGridMap[149][169] = 1;
_envmodle_info.ObstacleGridMap[150][169] = 1;
_envmodle_info.ObstacleGridMap[149][170] = 1;
_envmodle_info.ObstacleGridMap[150][170] = 1;
_envmodle_info.ObstacleGridMap[150][171] = 1;
_envmodle_info.ObstacleGridMap[150][172] = 1;
_envmodle_info.ObstacleGridMap[150][166] = 1;
_envmodle_info.ObstacleGridMap[150][167] = 1;



}

void get_Dt_RECORD_HdmapFrontPLane(Dt_RECORD_HdmapFrontPLane &_globePLane)
{

    // _globePLane.plan_seg_count = 2;
    // _globePLane.PlanSeg[0].Lane[0].node_count = 30;
    // _globePLane.PlanSeg[1].Lane[0].node_count = 30;

    // for(int idx_seg =0; idx_seg <_globePLane.plan_seg_count; ++idx_seg)
    // for(int idx_node =0; idx_node <_globePLane.PlanSeg[idx_seg].Lane[0].node_count; ++idx_node)
    // {
    //     _globePLane.PlanSeg[idx_seg].Lane[0].LaneNode[idx_node].hdmap_x = idx_seg *10 + idx_node;
    //     _globePLane.PlanSeg[idx_seg].Lane[0].LaneNode[idx_node].hdmap_y = idx_seg *5 + idx_node;
    // }

    _globePLane.plan_seg_count= 1;
    _globePLane.PlanSeg[0].Lane[0].lane_width = 3.6;
    // _globePLane.PlanSeg[1].Lane[0].lane_width = 0;
    // _globePLane.PlanSeg[2].Lane[0].lane_width = 5.6; 


// TEST 2021-01-23  to simulate Dealwith hdmap
_globePLane.PlanSeg[0].lane_count=1;
_globePLane.PlanSeg[0].Lane[0].node_count=22;

_globePLane.PlanSeg[0].Lane[0].LaneNode[0].hdmap_x =  83.5691;
_globePLane.PlanSeg[0].Lane[0].LaneNode[1].hdmap_x = 84.6033;
_globePLane.PlanSeg[0].Lane[0].LaneNode[2].hdmap_x = 85.6543;
_globePLane.PlanSeg[0].Lane[0].LaneNode[3].hdmap_x = 86.7181;
_globePLane.PlanSeg[0].Lane[0].LaneNode[4].hdmap_x = 87.7908;
_globePLane.PlanSeg[0].Lane[0].LaneNode[5].hdmap_x = 88.8687;
_globePLane.PlanSeg[0].Lane[0].LaneNode[6].hdmap_x = 89.9478;
_globePLane.PlanSeg[0].Lane[0].LaneNode[7].hdmap_x = 91.0244;
_globePLane.PlanSeg[0].Lane[0].LaneNode[8].hdmap_x = 92.0944;
_globePLane.PlanSeg[0].Lane[0].LaneNode[9].hdmap_x = 93.154;
_globePLane.PlanSeg[0].Lane[0].LaneNode[10].hdmap_x = 94.1995;
_globePLane.PlanSeg[0].Lane[0].LaneNode[11].hdmap_x = 95.2268;
_globePLane.PlanSeg[0].Lane[0].LaneNode[12].hdmap_x = 96.2323;
_globePLane.PlanSeg[0].Lane[0].LaneNode[13].hdmap_x = 97.2118;
_globePLane.PlanSeg[0].Lane[0].LaneNode[14].hdmap_x = 98.1617;
_globePLane.PlanSeg[0].Lane[0].LaneNode[15].hdmap_x = 99.0781;
_globePLane.PlanSeg[0].Lane[0].LaneNode[16].hdmap_x = 99.957;
_globePLane.PlanSeg[0].Lane[0].LaneNode[17].hdmap_x = 100.7947;
_globePLane.PlanSeg[0].Lane[0].LaneNode[18].hdmap_x = 101.5872;
_globePLane.PlanSeg[0].Lane[0].LaneNode[19].hdmap_x = 102.5669;
_globePLane.PlanSeg[0].Lane[0].LaneNode[20].hdmap_x = 103.4503;
_globePLane.PlanSeg[0].Lane[0].LaneNode[21].hdmap_x = 104.2283;

_globePLane.PlanSeg[0].Lane[0].LaneNode[0].hdmap_y=	102.7173;
_globePLane.PlanSeg[0].Lane[0].LaneNode[1].hdmap_y=	101.0485;
_globePLane.PlanSeg[0].Lane[0].LaneNode[2].hdmap_y=	99.3525;
_globePLane.PlanSeg[0].Lane[0].LaneNode[3].hdmap_y=	97.6356;
_globePLane.PlanSeg[0].Lane[0].LaneNode[4].hdmap_y=	95.904;
_globePLane.PlanSeg[0].Lane[0].LaneNode[5].hdmap_y=	94.1641;
_globePLane.PlanSeg[0].Lane[0].LaneNode[6].hdmap_y=	92.4219;
_globePLane.PlanSeg[0].Lane[0].LaneNode[7].hdmap_y=	90.6839;
_globePLane.PlanSeg[0].Lane[0].LaneNode[8].hdmap_y=	88.9561;
_globePLane.PlanSeg[0].Lane[0].LaneNode[9].hdmap_y=	87.245;
_globePLane.PlanSeg[0].Lane[0].LaneNode[10].hdmap_y=	85.5566;
_globePLane.PlanSeg[0].Lane[0].LaneNode[11].hdmap_y=	83.8974;
_globePLane.PlanSeg[0].Lane[0].LaneNode[12].hdmap_y=	82.2734;
_globePLane.PlanSeg[0].Lane[0].LaneNode[13].hdmap_y=	80.6911;
_globePLane.PlanSeg[0].Lane[0].LaneNode[14].hdmap_y=	79.1565;
_globePLane.PlanSeg[0].Lane[0].LaneNode[15].hdmap_y=	77.676;
_globePLane.PlanSeg[0].Lane[0].LaneNode[16].hdmap_y=	76.2558;
_globePLane.PlanSeg[0].Lane[0].LaneNode[17].hdmap_y=	74.9021;
_globePLane.PlanSeg[0].Lane[0].LaneNode[18].hdmap_y=	73.6213;
_globePLane.PlanSeg[0].Lane[0].LaneNode[19].hdmap_y=	72.0375;
_globePLane.PlanSeg[0].Lane[0].LaneNode[20].hdmap_y=	70.6091;
_globePLane.PlanSeg[0].Lane[0].LaneNode[21].hdmap_y=	69.3509;

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
        std::cout << "***************LOG FILE SAVE SUCCESSFULLY******** Frame size: " << logfile.frame_size() << std::endl;
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

void read_from_simulate_data(hdMapTrajectory &Trajectory, DecisionToPC &rev_DecisionToPC_data){
    laneInfo globalpath;
    // get_globalpath(globalpath);
    // get_hdMapTrajectory(Trajectory, globalpath);
    get_Dt_RECORD_EnvModelInfos(rev_DecisionToPC_data.my_envModelInfo);
    get_Dt_RECORD_HdmapFrontPLane(rev_DecisionToPC_data.my_hdmapFrontPLaneInfo);
    get_Dt_RECORD_LocalizationInfo(rev_DecisionToPC_data.my_localizationInfo);
    

}

void convet_trajectory(hdMapTrajectory &Trajectory, Dt_RECORD_TrajectoryPointsInfos &trajectoryPointsInfo){
        int node_=0;
        for(uint32 seg=0; seg<Trajectory.pathLane[0].segNum; ++seg)
        {
            for(uint32 node=0; node<Trajectory.pathLane[0].hdmapPathInfo[seg].laneInfos[0].nodeNum; ++node)
            {
                trajectoryPointsInfo.TrajectoryPoints[node_].x = Trajectory.pathLane[0].hdmapPathInfo[seg].laneInfos[0].laneNodeInfos[node].x;
                trajectoryPointsInfo.TrajectoryPoints[node_].y = Trajectory.pathLane[0].hdmapPathInfo[seg].laneInfos[0].laneNodeInfos[node].y;
                trajectoryPointsInfo.TrajectoryPoints[node_].theta =Trajectory.pathLane[0].hdmapPathInfo[seg].laneInfos[0].laneNodeInfos[node].heading;
                node_++;
            }
        }
        trajectoryPointsInfo.point_num = node_;
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
    DecisionToPC   rev_DecisionToPC_data;


    
    #ifdef  SOCKETON
    TCPClient sclient_zu5; // socket with  Vehicle
    
    #else
        // Socket with Simulation
        char buffer[sizeof(rev_DecisionToPC_data)];
        //创建套接字
        int sock = socket(AF_INET, SOCK_STREAM, 0); //向服务器（特定的IP和端口）发起请求
        struct sockaddr_in serv_addr;
        memset(&serv_addr, 0, sizeof(serv_addr)); //每个字节都用0填充
        serv_addr.sin_family = AF_INET; //使用IPv4地址
        serv_addr.sin_addr.s_addr = inet_addr("192.168.43.32"); //具体的IP地址
        serv_addr.sin_port = htons(8000); //端口
        connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr));//读取服务器传回的数据
    #endif

    // alogorithm data
    decision decision_obj;  
    StaticDecision static_decision_obj; 
    fusion_decision fusion_decision_obj;
    decision_info decisionToPlan;
<<<<<<< HEAD
    decisionToPlan.decision_command = LEFTAVOID; // temp 
=======

>>>>>>> feature/socketfromWin2021-0119
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
       

    // val to be updated real-time
    PlotItems plot_items;
    
    /* window position  Vehicle Coordinate Window*/
    plt::subplot2grid(2,3,0,0,2,1);
    plt::xlim(-10,10);
    plt::ylim(-10,40);

    // lanes
    plt::Plot plotObj_ref_line("lane_safe_left",  "y--");
    plt::Plot plotObj_safe_line_left(" lane_safe_left",  "y--");
    plt::Plot plotObj_safe_line_right("lane_safe_right",  "y--");
    plt::Plot plotObj_mid_lane_left_0(" lane_mid_left_0",  "r");
    plt::Plot plotObj_mid_lane_left_1(" lane_mid_left_1",  "r");
    plt::Plot plotObj_mid_lane_left_2(" lane_mid_left_2",  "r");
    plt::Plot plotObj_mid_lane_right_0(" lane_mid_right_0",  "r");
    plt::Plot plotObj_mid_lane_right_1(" lane_mid_right_1",  "r");
    plt::Plot plotObj_mid_lane_right_2(" lane_mid_right_2",  "r");

    // obstalce list point 
    plt::Plot plotObj_objList_point_all("objList_point_all",  "y.");//k.
    plt::Plot plotObj_objList_point_cipv_left("objList_point_cipv_left",  "r<");
    plt::Plot plotObj_objList_point_cipv_mid("ojList_point_cipv_mid",  "r^");
    plt::Plot plotObj_objList_point_cipv_right("ojList_point_cipv_right",  "r>");

    //grid
    plt::Plot plotObj_grid_static("grid_static",  "k.");
    plt::Plot plotObj_grid_dynamic("grid_dynamic",  "c.");

    //ego position
    plt::Plot plotObj_ego_position("ego_position",  "b+");
    

    /* window Posiotion Map Coordinate Window*/
    plt::subplot2grid(2,3,0,1,1,1);
    plt::xlim(-200,200);
    plt::ylim(-200,200);

    // refline in mapcoordi
    plt::Plot plotObj_refpath_mapCoordi_seg0("refpath_mapCoordi_seg0",  "y.");
    plt::Plot plotObj_refpath_mapCoordi_seg1("refpath_mapCoordi_seg1",  "y.");
    plt::Plot plotObj_refpath_mapCoordi_seg2("refpath_mapCoordi_seg2",  "y.");

     plt::Plot plotObj_refpath_local_mapCoordi("refpath_mapCoordi_local",  "b.");


    // ego_posiotion in mapcoordi
    plt::Plot plotObj_ego_posiotion_mapCoordi("ego_posiotion_mapCoordi",  "r+");



    /* window Posiotion global Map Window */
    plt::subplot2grid(2,3,1,1,1,1);
    plt::xlim(-200,200);
    plt::ylim(-200,200);

    // ego_posiotion in global map
    plt::Plot plotObj_ego_posiotion_globalmap("ego_posiotion_globalmap",  "ro");

    // start point in global map
    plt::Plot plotObj_start_point_globalmap("start_point_globalmap",  "rx");

    // end point in global map
    plt::Plot plotObj_end_point_globalmap("end_point_globalmap",  "rx");


    /* window Posiotion infos Window */
    plt::subplot2grid(2,3,0,2,2,1);
    plt::xlim(0,20);
    plt::ylim(0,20); 




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
                #ifdef  SOCKETON
                TCPClient sclient_zu5(static_cast<unsigned short int>(8001), "192.168.1.70");
                #endif
                glo_initsocket_lock =1;
            }
            // read from socket
            #ifdef  SOCKETON
                receive_zu5Andparse_socket(rev_DecisionToPC_data, sclient_zu5);
            #else
                int readLen=0;
                int readSize = read(sock, buffer, sizeof(buffer)-1);
                int sumLen=0;
                memcpy(&sumLen,buffer,4);
                printf("sumLen = %d\n",sumLen);
                if(sumLen>0)
                {
                    int len=0;
                    while(readLen<sumLen)
                    {
                        int curLen=0;
                        curLen=read(sock, buffer+len, sizeof(buffer)-1);
                        readLen+=curLen;
                        len=readLen;
                        printf("readLen = %d\n",readLen);
                    }

                }
                printf("Message form server: %s,readSize=%d \n", buffer,readSize);
                memcpy(&rev_DecisionToPC_data, buffer, sizeof(DecisionToPC));
            #endif

            if(MODE =="log") // Log Proto 
            {
                std::string file_name = static_cast<string>(argv[2]);

                // read_from_simulate_data(Trajectory, rev_DecisionToPC_data); /// temp should be del

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
                read_from_simulate_data(Trajectory, rev_DecisionToPC_data);
                if(app_stopped_req == true)
                    app_shutdown =true;   
            }
            
            if(app_stopped_req == true)
                app_shutdown =true;   
                 
        }

        /*    ASSIGN VALUES    */
        trajectoryPointsInfo = rev_DecisionToPC_data.my_trajectoryPointsInfo; // out2 next SWC
        globePLanehdmapInfos = rev_DecisionToPC_data.my_hdmapInfo;
        globePLane = rev_DecisionToPC_data.my_hdmapFrontPLaneInfo;
        localPLanne = rev_DecisionToPC_data.my_hdmapLocalLaneInfo;
        localInfos = rev_DecisionToPC_data.my_localizationInfo;
        envModelInfo = rev_DecisionToPC_data.my_envModelInfo;
        vehicleInfo = rev_DecisionToPC_data.my_vehicleInfo;

        std::cout << "current frame idx:" << frame_index <<std::endl;



        /*   ALGORITHMs  */
		decision_obj.DealWithHdmap(&globePLanehdmapInfos, localInfos.LocalizationResult.x, localInfos.LocalizationResult.y, localInfos.yaw, &globePLane, &localPLanne, &Trajectory);
        decision_obj.ObjDetect(1, &Trajectory, &globePLanehdmapInfos, &globePLane, &localPLanne, &localInfos, &envModelInfo, ego_config, &decisionToPlan, &selectObj);
        // convet_trajectory(Trajectory, trajectoryPointsInfo);
        // decision_obj.convert_flat_to_vehicle(&localInfos,globePLane);
        gridmap_coll_obj coll_obj;
        static_decision_obj.RouteObjDectbyGrid(localInfos.yaw, 1, &Trajectory, &globePLanehdmapInfos, &envModelInfo, &coll_obj, &globePLane, &localPLanne, &decisionToPlan);
        fusion_decision_obj.objSelectFusion(coll_obj.obj_mid_flag,      coll_obj.s_m, &selectObj.frontMid,     "laneMid     "); //midlane mid
        fusion_decision_obj.objSelectFusion(coll_obj.obj_midLeft_flag,  coll_obj.s_m, &selectObj.frontMidLeft, "laneMidLeft "); //midlane midleft
        fusion_decision_obj.objSelectFusion(coll_obj.obj_midRight_flag, coll_obj.s_m, &selectObj.frontMidRight,"laneMidRight"); //midlane mid
        fusion_decision_obj.objSelectFusion(coll_obj.obj_lef_flag,      coll_obj.s_l, &selectObj.frontLeft,    "laneLeft    "); //midlane Left
        fusion_decision_obj.objSelectFusion(coll_obj.obj_rig_flag,      coll_obj.s_r, &selectObj.frontRight,   "laneRight   "); //midlane Right




       
        /*   DISPALY    */
        // TO plot each Wind as one fun

        
        plot(rev_DecisionToPC_data, Trajectory, selectObj, plot_items);

        // update lines
        plotObj_safe_line_left.update(plot_items.safeline[0].x, plot_items.safeline[0].y);
        plotObj_safe_line_right.update(plot_items.safeline[1].x, plot_items.safeline[1].y);
        plotObj_mid_lane_left_0.update(plot_items.lane_midL[0].x, plot_items.lane_midL[0].y);
        plotObj_mid_lane_left_1.update(plot_items.lane_midL[1].x, plot_items.lane_midL[1].y);
        plotObj_mid_lane_left_2.update(plot_items.lane_midL[2].x, plot_items.lane_midL[2].y);
        plotObj_mid_lane_right_0.update(plot_items.lane_midR[0].x, plot_items.lane_midR[0].y);
        plotObj_mid_lane_right_1.update(plot_items.lane_midR[1].x, plot_items.lane_midR[1].y);
        plotObj_mid_lane_right_2.update(plot_items.lane_midR[2].x, plot_items.lane_midR[2].y);

        // update obstalce list
        plotObj_objList_point_all.update(plot_items.center_obstacle.x, plot_items.center_obstacle.y);
        plotObj_objList_point_cipv_left.update(plot_items.cipv[0].x, plot_items.cipv[0].y);
        plotObj_objList_point_cipv_mid.update(plot_items.cipv[1].x, plot_items.cipv[1].y);
        plotObj_objList_point_cipv_right.update(plot_items.cipv[2].x, plot_items.cipv[2].y);

        
        // update grid
        plotObj_grid_static.update(plot_items.grid[0].x, plot_items.grid[0].y);
        plotObj_grid_dynamic.update(plot_items.grid[1].x, plot_items.grid[1].y);

        // update ego position
        plotObj_ego_position.update(plot_items.ego_position.x, plot_items.ego_position.y);
       

        // update refpath_mapCoordi
        plotObj_refpath_mapCoordi_seg0.update(plot_items.refline_mapCoor[0].x, plot_items.refline_mapCoor[0].y);
        plotObj_refpath_mapCoordi_seg1.update(plot_items.refline_mapCoor[1].x, plot_items.refline_mapCoor[1].y);
        plotObj_refpath_mapCoordi_seg2.update(plot_items.refline_mapCoor[2].x, plot_items.refline_mapCoor[2].y);
        plotObj_refpath_local_mapCoordi.update(plot_items.refline_local_mapCoor.x, plot_items.refline_local_mapCoor.y);

        // update ego_posiotion_mapCoordi
        plotObj_ego_posiotion_mapCoordi.update(plot_items.ego_position_mapCoor.x, plot_items.ego_position_mapCoor.y);

        
        plotObj_ego_posiotion_globalmap.update(plot_items.ego_position_globalmap.x, plot_items.ego_position_globalmap.y);
        plotObj_start_point_globalmap.update(plot_items.start_point_globalmap.x, plot_items.start_point_globalmap.y);
        plotObj_end_point_globalmap.update(plot_items.end_point_globalmap.x, plot_items.end_point_globalmap.y);


        /* OTHER ITEMS */
        signal(SIGINT, sigint_handler);
        frame_index++;
        g_t_e = clock();
        std::cout << "----> frame: " << frame_index <<"  time cost:  " << (double)(g_t_e -g_t_s)/CLOCKS_PER_SEC << "s"<< std::endl;;
        if(app_shutdown ==true)
        {
            #ifndef SOCKETON
                close(sock); 
            #endif // !SOC
            
            return 0;
        }
    }
    #ifndef SOCKETON
        close(sock); 
    #endif // !SOC
    return 0;
}