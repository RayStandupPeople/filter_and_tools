#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>


#include "../common/libs/functions.h"
#include "../common/libs/types.h"
#include "../../obstacle_selection/libs/obstacle_selection.h"
#include "build/types.pb.h"
#include "build/obstacleSel.pb.h"

Path get_globalpath(){

 std::ifstream in_file("../../../log/geely.csv",std::ios::in);
    if(!in_file.is_open()){
        std::cout << "ERROR: OPEN  file "<<std::endl;
    }
    std::string line;
    Path _local_path;
    Node _node;
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
        _node.flat_X = x;
        _node.flat_Y = y;
        _node.Heading = Heading;
        // _node.S = s;
        _local_path.data.push_back(_node);

    }
    _local_path.id =0;
    return _local_path;
}

Path get_localpath(const Location &loc, const Path &glopath)
{
    int loc_idx =0;
    for(int i=0; i< glopath.data.size(); ++i)
    {
        if(loc.flat_X == glopath.data[i].flat_X && loc.flat_Y == glopath.data[i].flat_Y)
        loc_idx = i;
    }

    Path _localpath;
    for(int i = loc_idx - 10; i< loc_idx +20; ++i)
    {
        if(i < 0) continue;
        if(i >= glopath.data.size()) continue;

         _localpath.data.push_back(glopath.data[i]);
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

hdMapTrajectory get_hdMapTrajectory(const Path &globalpath){
    hdMapTrajectory _trajectory;
    memset(&_trajectory, 0, sizeof(hdMapTrajectory));
    _trajectory.pathLane[0].segNum = 1;
    _trajectory.pathLane[0].hdmapPathInfo[0].laneNum = 1; //no nearby lane
    _trajectory.pathLane[0].hdmapPathInfo[0].laneInfos[0].nodeNum = globalpath.data.size();
    // std::cout<<"_trajectory.pathLane[0].hdmapPathInfo[0].laneInfos[0].nodeNum:" <<globalpath.data.size();
    for(int node_idx=0; node_idx < globalpath.data.size(); ++node_idx)
    {
        laneNode _lane_node;
        memset(&_lane_node, 0, sizeof(laneNode));
        _lane_node.x = globalpath.data[node_idx].flat_X;
        _lane_node.y = globalpath.data[node_idx].flat_Y;
        _lane_node.heading = globalpath.data[node_idx].Heading;
        _trajectory.pathLane[0].hdmapPathInfo[0].laneInfos[0].laneNodeInfos.push_back(_lane_node);
    }
    return _trajectory;
}

Dt_RECORD_LocalizationInfo get_Dt_RECORD_LocalizationInfo(const Location &location)
{
    Dt_RECORD_LocalizationInfo _location;
    Dt_RECORD_LocalizationResult _location_res;
    memset(&_location, 0, sizeof(Dt_RECORD_LocalizationInfo));
    memset(&_location_res, 0, sizeof(Dt_RECORD_LocalizationResult));
    _location_res.x = location.flat_X;
    _location_res.y = location.flat_Y;
    _location.LocalizationResult = _location_res;
    _location.yaw = location.Heading;
    return _location;
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


void set_info_to_protobuf_file(std::vector<std::vector<Obj_sel>> &obj_list_vet, \
    const std::vector<Obj_sel> &cipv_obj_vet, const Path &path)
    {
    // //PRINT to terminal
    // for(int i=0;i<obj_list_vet.size();++i)
    // // int i =515;
    // {
    //     std::cout << "obs_num: " << obj_list_vet[i].size() << std::endl;
    //     for(int j=0;j<obj_list_vet[i].size();++j)
    //     {
    //         std::cout << "id = " << obj_list_vet[i][j].id <<std::endl;
    //         std::cout << "type = " << obj_list_vet[i][j].type <<std::endl;
    //         std::cout << "pos_x = " << obj_list_vet[i][j].pos_x <<std::endl;
    //         std::cout << "pos_y = " << obj_list_vet[i][j].pos_y <<std::endl;
    //         std::cout << "heading = " << obj_list_vet[i][j].heading <<std::endl;
    //         std::cout << "pos_s = " << obj_list_vet[i][j].pos_s <<std::endl;
    //         std::cout << "pos_d = " << obj_list_vet[i][j].pos_d <<std::endl;
    //         std::cout << std::endl;
    //     }
    // }


    // for(int i=0;i<path.data.size();++i)
    // {
    //     std::cout << "flat_X = "  << path.data[i].flat_X <<std::endl;
    //     std::cout << "flat_Y = "  << path.data[i].flat_Y <<std::endl;
    //     std::cout << "Heading = " << path.data[i].Heading <<std::endl;
    //     std::cout << "car_x = "   << path.data[i].car_x <<std::endl;
    //     std::cout << "car_y = "   << path.data[i].car_y <<std::endl;
    //     std::cout << "s = "       << path.data[i].s <<std::endl;
    //     std::cout << "d = "       << path.data[i].d <<std::endl;
    //     std::cout << std::endl;
    // }

    pb_obstacle_sel::LogFile logfile; 
    int frame_num = obj_list_vet.size();
    logfile.set_frame_total_num(frame_num);
    for(int index =0; index < frame_num; ++index )
    // int index =0; 
    {
        pb_obstacle_sel::Frame *frame_pb = logfile.add_frame();
        frame_pb->set_frame_id(index);

        pb_obstacle_sel::Path  *path_pb = new pb_obstacle_sel::Path();
        for(int i=0;i<path.data.size();++i)
        {
            pb_obstacle_sel::PathNode *data = path_pb->add_path_node();
            data->set_flat_x(path.data[i].flat_X);
            data->set_flat_y(path.data[i].flat_Y);
            data->set_car_x(path.data[i].car_x);
            data->set_car_y(path.data[i].car_y);
            data->set_s(path.data[i].s);
            data->set_d(path.data[i].d);
            data->set_heading(path.data[i].Heading);
            // std::cout<< path.data[i].Heading << std::endl;
        }
        frame_pb->set_allocated_path(path_pb);

        pb_obstacle_sel::ObstacleList *obstacle_list_pb = new pb_obstacle_sel::ObstacleList();
        int obstacle_num =0;
        for(int i=0;i<obj_list_vet[index].size();++i)
        {
            // std::cout << obj_list_vet[index].size() << std::endl;
            pb_obstacle_sel::Obstacle *data = obstacle_list_pb->add_obstacle();
            obstacle_num++;
            data->set_id(obj_list_vet[index][i].id);
            data->set_type(obj_list_vet[index][i].type);
            data->set_pos_x(obj_list_vet[index][i].pos_x);
            data->set_pos_y(obj_list_vet[index][i].pos_y);
            data->set_pos_s(obj_list_vet[index][i].pos_s);
            data->set_pos_d(obj_list_vet[index][i].pos_d);
        }
        obstacle_list_pb->set_num(obstacle_num);
        frame_pb->set_allocated_obstacle_list(obstacle_list_pb);

        pb_obstacle_sel::CIPVObstacle *cipv_obj_pb = new pb_obstacle_sel::CIPVObstacle();
        pb_obstacle_sel::Obstacle *obs_ = new pb_obstacle_sel::Obstacle();
        obs_->set_pos_x(cipv_obj_vet[index].pos_x);
        obs_->set_pos_y(cipv_obj_vet[index].pos_y);
        obs_->set_pos_s(cipv_obj_vet[index].pos_s);
        obs_->set_pos_d(cipv_obj_vet[index].pos_d);
        cipv_obj_pb->set_allocated_cipv_obstacle(obs_);
        frame_pb->set_allocated_cipv_obj(cipv_obj_pb);


        // std::cout<< frame_pb->cipv_obj().cipv_obstacle().pos_x() << std::endl;
        // if(cipv_obj_vet[index].pos_x!=0)
        // std::cout<<cipv_obj_vet[index].pos_x <<  " " << cipv_obj_vet[index].pos_y << std::endl;
    } 

    
    std::ofstream ofile("../data/data_convert",std::ios::out);
    if(!ofile.is_open())
        std::cout << "can not find data_convert file" << std::endl;
    logfile.SerializeToOstream(&ofile);
    ofile.close();
}


int main(int argc, const char** argv) {

    // get path
    Path globalpath = get_globalpath();

    // get location
    std::vector<Location> location_vet;
    // for(int i=0; i<globalpath.data.size(); ++i)
    // {
    //     Location loc;
    //     loc.flat_X  = globalpath.data[i].flat_X;
    //     loc.flat_Y  = globalpath.data[i].flat_Y;
    //     loc.Heading = globalpath.data[i].Heading;
    //     location_vet.push_back(loc); 
    // }	
    Location loc;
    loc.flat_X = -65.2941812771520;	
    loc.flat_Y = -171.744892903921;
    loc.Heading = 270.400597767095;
    location_vet.push_back(loc);

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
        memset(&envModelInfo,0,sizeof(Dt_RECORD_EnvModelInfos));    
        memset(&selectObj,0,sizeof(objSecList));
        /* init Input */

       
        hdMapTrajectory hdMapTrajectory_ = get_hdMapTrajectory(globalpath);
        Dt_RECORD_LocalizationInfo LocalizationInfo_ = get_Dt_RECORD_LocalizationInfo(location_vet[0]);
        Dt_RECORD_EnvModelInfos EnvModelInfos_ = get_Dt_RECORD_EnvModelInfos(obj_list_vet[frame_idx]);//frame_idx = 515 valid
        decision_obj.ObjDetect(1, &hdMapTrajectory_, &hdmapInfos, &globePLane, &localPLanne, &LocalizationInfo_, 
            &EnvModelInfos_, ego_config, &selectObj);
        
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
        _cipv_obj.pos_s = selectObj.frontMid.obj.pos_s;
        _cipv_obj.pos_d = selectObj.frontMid.obj.pos_d;
        cipv_obj_vet.push_back(_cipv_obj);
        // std::cout << cipv_obj_vet.size() << std::endl;
        // std::cout <<  _cipv_obj.id  << " " << _cipv_obj.pos_x<<" " <<  _cipv_obj.pos_y <<std::endl;
        /*  for save CIPV Protobuf */

    }
    set_info_to_protobuf_file(obj_list_vet, cipv_obj_vet, globalpath);
   
    std::cout << "ok";
    return 0;
}