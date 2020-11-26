#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>


#include "../common/libs/functions.h"
#include "../common/libs/types.h"
#include "build/types.pb.h"
#include "build/obstacleSel.pb.h"

Path get_localpath(){

 std::ifstream in_file("../../../log/geely.csv",std::ios::in);
    if(!in_file.is_open()){
        std::cout << "ERROR: OPEN  file "<<std::endl;
    }
    std::string line;
    Path _local_path;
    Node _node;
    double x;
  	double y;
  	double s;
  	double d_x;
  	double d_y;
    std::vector<double> x_array;
    std::vector<double> y_array;
    std::vector<double> s_array;

    while(!in_file.eof()){
        getline(in_file, line);
        std::istringstream iss(line);
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        // x_array.push_back(x);
        // y_array.push_back(y);
        // s_array.push_back(s);
        _node.flat_X = x;
        _node.flat_Y = y;
        // _node.S = s;
        _local_path.data.push_back(_node);

    }
    _local_path.id =0;
    return _local_path;
}


std::vector<obj_sel> get_obstalce_lists(const int &frame_id){
    std::ifstream in_file("../../../log/wholeFile_info_pb",std::ios::in);
    if(!in_file.is_open()){
        std::cout << "ERROR: OPEN  file "<<std::endl;
    }
    std::stringstream  CodeStrstream;
    pb_types::LogFile logfile;
    CodeStrstream << in_file.rdbuf();
    in_file.close();
    logfile.ParseFromString(CodeStrstream.str());

    std::vector<obj_sel> _obstacle_list;
        obj_sel _obstacle_sel;

    for(int j=0;j<logfile.frame(frame_id).obstacle_size();++j)
    {
            pb_types::Obstacle _obstacle = logfile.frame(frame_id).obstacle(j);
            _obstacle_sel.id      = _obstacle.id();
            _obstacle_sel.type    = _obstacle.type();
            _obstacle_sel.pos_x   = _obstacle.pos_x();
            _obstacle_sel.pos_y   = _obstacle.pos_y();
            _obstacle_list.push_back(_obstacle_sel);
    }
    return _obstacle_list;
}


void convert_coordinate_flat_to_vehicle(const Location &location, Path &localpath){

    double a = location.flat_X;
    double b = location.flat_Y;
    double t = -location.Heading * M_PI/180;

    for(int i=0;i<localpath.data.size();++i)
    {
        localpath.data[i].car_x = (localpath.data[i].flat_X - a)*cos(t)   + (localpath.data[i].flat_Y - b)*sin(t);
        localpath.data[i].car_y = (localpath.data[i].flat_X - a)*-sin(t)  + (localpath.data[i].flat_Y - b)*cos(t);
    }

}

void convert_coordinate_cartesian_to_frenet(const Location &location, Path &path){
    std::vector<double> _X;
    std::vector<double> _Y;
    std::vector<double> sd_={0,0};
    for(int i=0;i<path.data.size();++i)
    {
        _X.push_back(path.data[i].car_x);
        _Y.push_back(path.data[i].car_y);
    }

    double egoVeh_s = getFrenet(location.car_x, location.car_y, location.Heading, _X, _Y)[0];

    for(int i=0;i<path.data.size();++i)
    {
        sd_ = getFrenet(path.data[i].car_x,path.data[i].car_y,path.data[i].Heading,_X, _Y);
        path.data[i].s = sd_[0] - egoVeh_s; 
    }
    
    // // PRINT
    // for(int i=0;i<path.data.size();++i)
    //     std::cout << path.data[i].s << " " <<  path.data[i].d << std::endl;
    // std::cout << std::endl;

}

void convert_coordinate_cartesian_to_frenet(std::vector<obj_sel> &obj_list, Path &path)
{
    std::vector<double> _X;
    std::vector<double> _Y;
    std::vector<double> _Obj_X;
    std::vector<double> _Obj_Y;
    std::vector<double> sd_={0,0};
    for(int i=0;i<path.data.size();++i)
    {
        _X.push_back(path.data[i].car_x);
        _Y.push_back(path.data[i].car_y);
    }
    
    // get origin frenet S
    double ori_s = getFrenet(0, 0, 0,_X, _Y)[0];

    for(int i=0;i<obj_list.size();++i)
    {
        sd_ = getFrenet(-obj_list[i].pos_y, obj_list[i].pos_x, 0,_X, _Y);
        obj_list[i].pos_s = sd_[0] - ori_s; 
        obj_list[i].pos_d = sd_[1]; 
    }

    // // PRINT
    // for(int i=0;i<obj_list.size();++i)
    //     std::cout << obj_list[i].pos_s << " " << obj_list[i].pos_d << std::endl;
    // std::cout << std::endl;
}
void set_info_to_protobuf_file(std::vector<obj_sel> &obj_list, const Path &path){
    // //PRINT to terminal
    // for(int i=0;i<obj_list.size();++i)
    // {
    //     std::cout << "id = " << obj_list[i].id <<std::endl;
    //     std::cout << "type = " << obj_list[i].type <<std::endl;
    //     std::cout << "pos_x = " << obj_list[i].pos_x <<std::endl;
    //     std::cout << "pos_y = " << obj_list[i].pos_y <<std::endl;
    //     std::cout << "heading = " << obj_list[i].heading <<std::endl;
    //     std::cout << "pos_s = " << obj_list[i].pos_s <<std::endl;
    //     std::cout << "pos_d = " << obj_list[i].pos_d <<std::endl;
    //     std::cout << std::endl;
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
    pb_obstacle_sel::Path *path_pb = new pb_obstacle_sel::Path();
    for(int i=0;i<path.data.size();++i)
    {
        pb_obstacle_sel::PathNode *data = path_pb->add_path_node();
        data->set_flat_x(path.data[i].flat_X);
        data->set_flat_y(path.data[i].flat_Y);
        data->set_car_x(path.data[i].car_x);
        data->set_car_y(path.data[i].car_y);
        data->set_s(path.data[i].s);
        data->set_d(path.data[i].d);
    }
    logfile.set_allocated_path(path_pb);


    pb_obstacle_sel::ObstacleList *obstacle_list_pb = new pb_obstacle_sel::ObstacleList();
    for(int i=0;i<obj_list.size();++i)
    {
        pb_obstacle_sel::Obstacle *data = obstacle_list_pb->add_obstacle();
        data->set_id(obj_list[i].id);
        data->set_type(obj_list[i].type);
        data->set_pos_x(obj_list[i].pos_x);
        data->set_pos_y(obj_list[i].pos_y);
        data->set_pos_s(obj_list[i].pos_s);
        data->set_pos_d(obj_list[i].pos_d);
    }

    logfile.set_allocated_obstacle_list(obstacle_list_pb);


    // std::cout << logfile.obstacle_list().obstacle_size();
    // std::cout << logfile.path().path_node_size();

    std::ofstream ofile("../data/data_convert",std::ios::out);
    if(!ofile.is_open())
        std::cout << "can not find data_convert file" << std::endl;
    logfile.SerializeToOstream(&ofile);
    ofile.close();
}

void cipv_selection(std::vector<obj_sel> &obj_list, obj_sel &cipv_obj){
    double lane_width = 2.7;
    double min_dis = 150;
    for(int i=0; i< obj_list.size(); ++i)
    {
        if(obj_list[i].pos_d > -lane_width/2 && obj_list[i].pos_d < lane_width/2 )
        {
            if(obj_list[i].pos_s < min_dis  &&  obj_list[i].pos_s >0 )
            {
                min_dis = obj_list[i].pos_s;
                cipv_obj.id = obj_list[i].id;
            }
        }
    }
    for(int i=0; i< obj_list.size(); ++i)
    {
        if(cipv_obj.id == obj_list[i].id)
            cipv_obj = obj_list[i];
    }
}

int main(int argc, const char** argv) {
    Path localpath = get_localpath();
    Location location;		
        location.flat_X = -53.9239494768775;
        location.flat_Y = -171.757296549627;
        location.Heading = 280.446772134423;
        // -67.3449392315137	-171.730554283322	274.346816390618
    obj_sel cipv_obj;
    std::vector<obj_sel> obj_list = get_obstalce_lists(1);
    convert_coordinate_flat_to_vehicle(location, localpath);
    convert_coordinate_cartesian_to_frenet(location, localpath);
    convert_coordinate_cartesian_to_frenet(obj_list, localpath);
    set_info_to_protobuf_file(obj_list, localpath);
    cipv_selection(obj_list, cipv_obj);
    std::cout << cipv_obj.id << " " << cipv_obj.pos_s << " " << cipv_obj.pos_d << std::endl;
    return 0;
}
