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


#define OFFSET(st, field)     (size_t)&(((st*)0)->field)
uint32 glo_initsocket_lock; // global value which for initing socket


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
        socket_data_deserialization(sclient_zu5, rev_DecisionToPC_data, id, timestamp);
    }
    else
    {
        sclient_zu5.close();
        sclient_zu5.reconnect(8001, "192.168.1.70");
    }
}
void log_protobuf_file(HdmapToPc_data &rev_hdmapToPC_data, DecisionToPC &rev_DecisionToPC_data,\    
     uint32 index, socket_all::LogFile &logfile)
{

        logfile.set_frame_total_num(index);
        socket_all::Frame *frame_pb = logfile.add_frame();
        frame_pb->set_frame_id(index);

        socket_all::HdmapToPc_data  *HdmapToPc_data_pb = new socket_all::HdmapToPc_data();

        socket_all::Dt_RECORD_HdmapInfo *Dt_RECORD_HdmapInfo_pb = new socket_all::Dt_RECORD_HdmapInfo();
        Dt_RECORD_HdmapInfo_pb->set_planpath(rev_hdmapToPC_data.G_HdmapInfo.planpath);
        Dt_RECORD_HdmapInfo_pb->set_origin_x(rev_hdmapToPC_data.G_HdmapInfo.origin_x);
        Dt_RECORD_HdmapInfo_pb->set_origin_y(rev_hdmapToPC_data.G_HdmapInfo.origin_y);
        Dt_RECORD_HdmapInfo_pb->set_origin_z(rev_hdmapToPC_data.G_HdmapInfo.origin_z);
        Dt_RECORD_HdmapInfo_pb->set_origin_yaw(rev_hdmapToPC_data.G_HdmapInfo.origin_yaw);
        Dt_RECORD_HdmapInfo_pb->set_goal_x(rev_hdmapToPC_data.G_HdmapInfo.goal_x);
        Dt_RECORD_HdmapInfo_pb->set_goal_y(rev_hdmapToPC_data.G_HdmapInfo.goal_y);
        Dt_RECORD_HdmapInfo_pb->set_goal_z(rev_hdmapToPC_data.G_HdmapInfo.goal_z);
        Dt_RECORD_HdmapInfo_pb->set_goal_yaw(rev_hdmapToPC_data.G_HdmapInfo.goal_yaw);
        HdmapToPc_data_pb->set_allocated_g_hdmapinfo(Dt_RECORD_HdmapInfo_pb);
          
        
        frame_pb->set_allocated_hdmaptopc_data(HdmapToPc_data_pb);
               

        // socket_all::ObstacleList *obstacle_list_pb = new pb_obstacle_sel::ObstacleList();
        // int obstacle_num =0;
        // for(int i=0;i<obj_list_vet[index].size();++i)
        // {
        //     // std::cout << obj_list_vet[index].size() << std::endl;
        //     socket_all::Obstacle *data = obstacle_list_pb->add_obstacle();
        //     obstacle_num++;
        //     data->set_id(obj_list_vet[index][i].id);
        //     data->set_type(obj_list_vet[index][i].type);
        //     data->set_pos_x(obj_list_vet[index][i].pos_x);
        //     data->set_pos_y(obj_list_vet[index][i].pos_y);
        //     data->set_pos_s(obj_list_vet[index][i].pos_s);
        //     data->set_pos_d(obj_list_vet[index][i].pos_d);
        // }
        // obstacle_list_pb->set_num(obstacle_num);
        // frame_pb->set_allocated_obstacle_list(obstacle_list_pb);
        std::cout << "index:" <<index << std::endl;
    if(index>5)
    {
        std::ofstream ofile("../data/data_convert",std::ios::out);
            if(!ofile.is_open())
                std::cout << "can not find data_convert file" << std::endl;
            logfile.SerializeToOstream(&ofile);
            ofile.close();
    }
    
}
void read_from_proto_file(HdmapToPc_data &rev_hdmapToPC_data, DecisionToPC &rev_DecisionToPC_data){
    std::ifstream in_file("../data/data_convert",std::ios::in);
    if(!in_file.is_open()){
        std::cout << "ERROR: OPEN  file "<<std::endl;
    }
    std::stringstream  CodeStrstream;
    socket_all::LogFile logfile;
    CodeStrstream << in_file.rdbuf();
    in_file.close();
    logfile.ParseFromString(CodeStrstream.str());
    std::cout << "frame num:" <<logfile.frame_total_num() << std::endl;
   
    socket_all::Frame frame_pb = logfile.frame(3);
    socket_all::HdmapToPc_data HdmapToPc_data_pb =frame_pb.hdmaptopc_data();
    // std::cout << "onpayh" <<HdmapToPc_data_pb.g_hdmapinfo().planpath();
    rev_hdmapToPC_data.G_HdmapInfo.planpath = HdmapToPc_data_pb.g_hdmapinfo().planpath();
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

    /* DATA Define and init */
    // sensors data
    hdMapTrajectory Trajectory;
    Dt_RECORD_HdmapInfo globePLanehdmapInfos;
    Dt_RECORD_HdmapFrontPLane globePLane;
    Dt_RECORD_HdmapLocalLane localPLanne;
    Dt_RECORD_LocalizationInfo localInfos;
    Dt_RECORD_EnvModelInfos envModelInfo;
    EgoConfigPara ego_config;
    objSecList selectObj;
    memset(&Trajectory,0,sizeof(hdMapTrajectory));
    memset(&globePLanehdmapInfos,0,sizeof(Dt_RECORD_HdmapInfo));
    memset(&globePLane,0,sizeof(Dt_RECORD_HdmapFrontPLane));
    memset(&localPLanne,0,sizeof(Dt_RECORD_HdmapLocalLane));
    memset(&localInfos,0,sizeof(Dt_RECORD_LocalizationInfo));
    memset(&envModelInfo,0,sizeof(Dt_RECORD_EnvModelInfos));    
    memset(&selectObj,0,sizeof(objSecList));

    // socket data
    HdmapToPc_data rev_hdmapToPC_data;
    DecisionToPC   rev_DecisionToPC_data;
    TCPClient sclient_zu2;
    TCPClient sclient_zu5;

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
        MODE = static_cast<string>(argv[1]);
    
    uint32 frame_index =0; //logfile frame index
    socket_all::LogFile logfile; //logfile obj
       
    // CYCLE STRUCT
    while(true)
    {
        
        /* PARSE: form socket or file*/ 
        if(MODE == "pure" || MODE == "log") // Pure, Log  (ONline)
        {
            // // init socket
            // if(glo_initsocket_lock ==0)
            // {
            //     std::cout << "connecting socket..." << std::endl;
            //     TCPClient sclient_zu2(8001, "192.168.1.60");
            //     TCPClient sclient_zu5(8001, "192.168.1.70");
            //     glo_initsocket_lock =1;
            // }
            // // read from socket
            // thread socket_thread_zu2(receive_zu2Andparse_socket, std::ref(rev_hdmapToPC_data), std::ref(sclient_zu2));
            // thread socket_thread_zu5(receive_zu5Andparse_socket, std::ref(rev_DecisionToPC_data),std::ref(sclient_zu5));
            // socket_thread_zu2.join();
            // socket_thread_zu5.join();
            if(MODE =="log") // Log Proto 
            {
                std::cout << "Mode: log, display and log" << std::endl;
                log_protobuf_file(rev_hdmapToPC_data,rev_DecisionToPC_data, frame_index, logfile);
            }

        }
        if(MODE == "replay") // Replay  (OFF line)
        {
            std::cout << "Mode: replay, display proto file data" << std::endl;
            // read from proto
            // assign value
            read_from_proto_file(rev_hdmapToPC_data, rev_DecisionToPC_data);      
        }



        /*    ASSIGN VALUES    */
        laneInfo globalpath;
        get_globalpath(globalpath);
        get_hdMapTrajectory(Trajectory, globalpath);
        // Trajectory = rev_DecisionToPC_data.my_trajectoryPointsInfo;
        globePLanehdmapInfos = rev_hdmapToPC_data.G_HdmapInfo;
        globePLane = rev_hdmapToPC_data.G_FrontPLane;
        localPLanne = rev_hdmapToPC_data.G_LocalLane;
        localInfos = rev_DecisionToPC_data.my_localizationInfo;
        // envModelInfo = rev_DecisionToPC_data. ???
        get_Dt_RECORD_EnvModelInfos(envModelInfo);



        /*   ALGORITHMs  */
        decision_obj.ObjDetect(1, &Trajectory, &globePLanehdmapInfos, &globePLane, &localPLanne, &localInfos, &envModelInfo, ego_config, &selectObj);
        

        /*   DISPALY    */
        plot(Trajectory, globePLanehdmapInfos, globePLane, localInfos, envModelInfo, selectObj);
        frame_index++;
    }
    
    return 0;
}