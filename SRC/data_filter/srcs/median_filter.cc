#include<iostream>
#include<vector>
#include<fstream>
#include<sstream>
#include<memory>
#include "../libs/median_filter.h"
#include "../../common/libs/types.pb.h"
#include "../../common/libs/types.h"

std::vector<std::vector<double>> MedianFilter::data_windows(20); //apply Memory for windows 

/*
@breaf taget selection base on front rectangle
@param obstacle_list: original obstacle list
@param selected_obj_list: the selected obstacle which affects the ego car most
*/
void MedianFilter::obstacle_select(const std::vector<pb_types::Obstacle> &obstacle, objSecList *selected_obj_list){
    std::vector<pb_types::Obstacle> obj_in_field;
    int tar_obs_idx=-1;
    double _min_X =999;

    for(int i=0; i<obstacle.size();++i)
    {
        if(obstacle[i].pos_x() > 0.5 && obstacle[i].pos_x() < 30 && \
            obstacle[i].pos_y() > -1.7 && obstacle[i].pos_y() < 1.7)
        obj_in_field.push_back(obstacle[i]);

        for(int i=0; i<obj_in_field.size(); ++i)
        {
            if(obj_in_field[i].pos_x() < _min_X)
            {
                _min_X = obj_in_field[i].pos_x();
                tar_obs_idx = i;
            }
        }
        selected_obj_list->frontMid.obj.id = obj_in_field[tar_obs_idx].id();
        selected_obj_list->frontMid.obj.pos_x = obj_in_field[tar_obs_idx].pos_x();
        selected_obj_list->frontMid.obj.pos_y = obj_in_field[tar_obs_idx].pos_y();
    }
}

/*
@breaf typical moving window filters
@param data: original data which need to be filted
@param filter_type: 0-> midian filter,   1-> average filter
@param wind_size:  set moving window size, which is related to filter result
@param window_idx: index of specificed fitler windows, diffenet Obstacle property should with different window index
*/
void MedianFilter::move_window_filter(double &data, const char &filter_type, const size_t &wind_size, char window_idx){
    double wind_ave=0;
    double wind_mid=0;
    // static std::vector<double> data_wind; 
    data_windows[window_idx].push_back(data);
    //std::cout<<"[TEST] size" <<data_wind.size() << std::endl;
    if(data_windows[window_idx].size()<wind_size)
        return;
    if(data_windows[window_idx].size()>wind_size) // keep window moving
    {
        for(int i=0;i<wind_size;++i)
            std::swap(data_windows[window_idx][i],data_windows[window_idx][i+1]);
        data_windows[window_idx].erase(data_windows[window_idx].end()-1); //delete back element
    }
    double tol_ =0;
    for(int i=0;i<wind_size;++i)
        tol_+=data_windows[window_idx][i];
    wind_ave = tol_/wind_size;
   
    // PUPPLE range
    std::vector<double> data_wind_(data_windows[window_idx].begin(),data_windows[window_idx].end());
    for(int i=0;i<wind_size;++i)
    for(int j=0;j<wind_size-i-1;++j)
    if(data_wind_[j]>data_wind_[j+1])
        std::swap(data_wind_[j],data_wind_[j+1]);
    wind_mid = data_wind_[wind_size/2];
    
    if(filter_type==0)
        data = wind_mid;
    if(filter_type==1)
        data = wind_ave;
}

/*
@breaf:
@param filter_type: 0-> midian filter,   1-> average filter
@param wind_size:  set moving window size, which is related to filter result
*/
void MedianFilter::data_stabilization(objSecList *selected_obj_list, const char &filter_type, const size_t &wind_size){

    move_window_filter(selected_obj_list->frontMid.obj.pos_x,filter_type,wind_size,0);
    move_window_filter(selected_obj_list->frontMid.obj.pos_y,filter_type,wind_size,1);
//    move_window_filter(selected_obj_list->frontMid.obj.heading,filter_type,wind_size,2);
//    move_window_filter(selected_obj_list->frontMid.obj.rel_speed_x,filter_type,wind_size,3);
//    move_window_filter(selected_obj_list->frontMid.obj.rel_speed_y,filter_type,wind_size,4);
}

