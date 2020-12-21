#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>

#include "../common/libs/user_struct.h"
#include "../../obstacle_selection/libs/obstacle_selection.h"
#include "../common/libs/display.h"

#include "build/types.pb.h"
#include "build/obstacleSel.pb.h"

laneInfo get_globalpath(){

 std::ifstream in_file("../../../log/lg_curv.csv",std::ios::in);
    if(!in_file.is_open()){
        std::cout << "ERROR: OPEN  file "<<std::endl;
    }
    std::string line;
    laneInfo _local_path;
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
        _local_path.laneNodeInfos.push_back(_node);

    }
    _local_path.nodeNum = _local_path.laneNodeInfos.size();
    return _local_path;
}

laneInfo get_localpath(const Dt_RECORD_LocalizationResult &loc, const laneInfo &glopath)
{
    int loc_idx =0;
    for(int i=0; i< glopath.laneNodeInfos.size(); ++i)
    {
        if(loc.x == glopath.laneNodeInfos[i].x && loc.y == glopath.laneNodeInfos[i].y)
        loc_idx = i;
    }

    laneInfo _localpath;
    for(int i = loc_idx - 10; i< loc_idx +20; ++i)
    {
        if(i < 0) continue;
        if(i >= glopath.laneNodeInfos.size()) continue;

         _localpath.laneNodeInfos.push_back(glopath.laneNodeInfos[i]);
    }

    return _localpath;
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

hdMapTrajectory get_hdMapTrajectory(const laneInfo &globalpath){
    hdMapTrajectory _trajectory;
    memset(&_trajectory, 0, sizeof(hdMapTrajectory));
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
    return _trajectory;
}

void get_Dt_RECORD_LocalizationInfo( Dt_RECORD_LocalizationInfo &location, int i)
{
  
    location.LocalizationResult.x = -20 - i;
    location.LocalizationResult.y = 8;
    location.yaw = 10;
}

Dt_RECORD_EnvModelInfos get_Dt_RECORD_EnvModelInfos(const std::vector<Obj_sel> &obj_list)
{
    Dt_RECORD_EnvModelInfos _envmodle_info;
    memset(&_envmodle_info, 0, sizeof(Dt_RECORD_EnvModelInfos));
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
    return _envmodle_info;
}

void get_Dt_RECORD_HdmapFrontPLane(Dt_RECORD_HdmapFrontPLane &_globePLane)
{

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



int main(int argc, const char** argv) {

    // get path
    laneInfo globalpath = get_globalpath();

    // get objlist
    std::vector<std::vector<Obj_sel>> obj_list_vet = get_obstalce_lists();

    //init cipv    
    std::vector<Obj_sel> cipv_obj_vet; 
    

    // MAIN LOGIC
    decision decision_obj;   //new object
    for(int frame_idx=0; frame_idx<obj_list_vet.size(); ++frame_idx )
    {
            /* init INPUT */
        hdMapTrajectory Trajectory;
        Dt_RECORD_HdmapInfo hdmapInfos;
        Dt_RECORD_HdmapFrontPLane globePLane;
        Dt_RECORD_HdmapLocalLane localPLanne;
        Dt_RECORD_LocalizationInfo localInfos;
        Dt_RECORD_EnvModelInfos envModelInfo;
        EgoConfigPara ego_config;
        objSecList selectObj;
        memset(&Trajectory,0,sizeof(hdMapTrajectory));
        memset(&hdmapInfos,0,sizeof(Dt_RECORD_HdmapInfo));
        memset(&globePLane,0,sizeof(Dt_RECORD_HdmapFrontPLane));
        memset(&localPLanne,0,sizeof(Dt_RECORD_HdmapLocalLane));
        memset(&localInfos,0,sizeof(Dt_RECORD_LocalizationInfo));
        memset(&envModelInfo,0,sizeof(Dt_RECORD_EnvModelInfos));    
        memset(&selectObj,0,sizeof(objSecList));
        

        /* init Input */
        get_Dt_RECORD_LocalizationInfo(localInfos,frame_idx);
        Trajectory = get_hdMapTrajectory(globalpath);
        get_Dt_RECORD_HdmapFrontPLane(globePLane);
        envModelInfo = get_Dt_RECORD_EnvModelInfos(obj_list_vet[frame_idx]);//frame_idx = 515 valid
        // std::cout <<"num: " << envModelInfo.obstacle_num <<std::endl;
        decision_obj.ObjDetect(1, &Trajectory, &hdmapInfos, &globePLane, &localPLanne, &localInfos, 
            &envModelInfo, ego_config, &selectObj);
        // std::cout << "loc" << localInfos.LocalizationResult.x << localInfos.LocalizationResult.y << std::endl;

        // /*  for save CIPV Protobuf */
        // std::cout << "if exist cipv_front: " << selectObj.frontMid.postion << std::endl;
        // if(selectObj.frontMid.postion)
        //     std::cout << "cipv_front_id : " << selectObj.frontMid.obj.id << std::endl;
        // else
        //     std::cout << "No CIPV: " << selectObj.frontMid.obj.id << std::endl;
        Obj_sel _cipv_obj;
        _cipv_obj.id = selectObj.frontMid.obj.id;
        _cipv_obj.pos_x = selectObj.frontMid.obj.pos_x;
        _cipv_obj.pos_y = selectObj.frontMid.obj.pos_y;
        _cipv_obj.s = selectObj.frontMid.obj.s;
        _cipv_obj.d = selectObj.frontMid.obj.d;
        cipv_obj_vet.push_back(_cipv_obj);

        // // test
        // _cipv_obj.pos_x = frame_idx+10;
        // _cipv_obj.pos_y = 0.7;
        // //test

        // std::cout << cipv_obj_vet.size() << std::endl;
        // std::cout <<  _cipv_obj.id  << " " << _cipv_obj.pos_x<<" " <<  _cipv_obj.pos_y <<std::endl;
        /*  for save CIPV Protobuf */
        plot(Trajectory, hdmapInfos, globePLane, localInfos, envModelInfo, selectObj);
        // sleep(0.1);

    }
    
    std::cout << "ok";
    return 0;
}