#include<iostream>
#include<vector>
#include<fstream>
#include<sstream>
#include "../LIB/median_filter.h"


// #define VIRTUAL_DATA

std::vector<std::vector<double>> Solution::data_windows(20); //apply Memory for windows 

/*
@breaf typical moving window filters
@param data: original data which need to be filted
@param filter_type: 0-> midian filter,   1-> average filter
@param wind_size:  set moving window size, which is related to filter result
@param window_idx: index of specificed fitler windows, diffenet Obstacle property should with different window index
*/
void Solution::move_window_filter(double &data, const char &filter_type, const size_t &wind_size, char window_idx){
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
@breaf stablize interested property data of selected obstalces 
@param selected_obj_list: original data of selected object list struct
@param filter_type: 0-> midian filter,   1-> average filter
@param wind_size:  set moving window size, which is related to filter result
*/
void Solution::data_stabilization(objSecList *selected_obj_list, const char &filter_type, const size_t &wind_size){

    move_window_filter(selected_obj_list->frontMid.obj.pos_x,filter_type,wind_size,0);
    move_window_filter(selected_obj_list->frontMid.obj.pos_y,filter_type,wind_size,1);
//    move_window_filter(selected_obj_list->frontMid.obj.heading,filter_type,wind_size,2);
//    move_window_filter(selected_obj_list->frontMid.obj.rel_speed_x,filter_type,wind_size,3);
//    move_window_filter(selected_obj_list->frontMid.obj.rel_speed_y,filter_type,wind_size,4);
}

int main(){
    Solution s;
    objSecList obj_sel_list;
    std::vector<std::vector<double>> ori_data(2); // random nums
    std::vector<std::vector<double>> filted_num(2);  // rfilted nums
    std::ifstream in_file;
    std::ofstream out_file; 

    //* SWITCH ON : USE VIRTUAL DATA*
# ifdef VIRTUAL_DATA

    int times = 50;  // set simulation data length
    std::vector<std::vector<double>> rand_num(2); // random nums
    // high frequency random
    for(int i=0;i<times;++i)
    {
        rand_num[0].push_back(rand()%20);
        // rand2.push_back(rand()%20);
    }

    // low frequency random
    for(int i=0;i<times;++i)
    {
        double ran = rand()%100;
        if(ran > 90)
            rand_num[1].push_back(ran* 0.7);
        else
            rand_num[1].push_back(20+rand()%5);
    }
    ori_data = rand_num;
    //* SWITCH OFF : USE REAL FILE DATA*
#else
    std::string line_str;
    std::string word_str;
    in_file.open("../LOG/data_convert",std::ios::in);
    if(!in_file.is_open())
    {
        std::cout << "file open fail(read)" << std::endl;
    }
    for(int i =0;i<2;++i)
    {
        std::getline(in_file,line_str);
        std::stringstream linestream(line_str);
        while(linestream >> word_str)
            ori_data[i].push_back(stod(word_str));
    }
    out_file.close();
    int ele_size = ori_data[0].size();
    if(ele_size == 0)
    {
        std::cout << "READ FILE ERROR, EMPTY FILE" << std::endl;
        return -1;
    }
    for(int i=1; i<2;++i)
    {
        if(ori_data[i].size()!=ele_size)
        {
            std::cout << "READ FILE ERROR, element do NOT have the same dimension" << std::endl;
            return -1;
        }
    }
#endif

    
    /* DATA PROCESSION*/
    for(int i=0;i<ori_data[0].size();++i)
    {
        obj_sel_list.frontMid.obj.pos_x = ori_data[0][i];
        obj_sel_list.frontMid.obj.pos_y = ori_data[1][i];
        s.data_stabilization(&obj_sel_list,0,10); // ** KEY FUN **
        filted_num[0].push_back(obj_sel_list.frontMid.obj.pos_x);
        filted_num[1].push_back(obj_sel_list.frontMid.obj.pos_y);
    }


    /* DATA OUTPUT*/
    out_file.open("../LOG/data_convert",std::ios::out);
        if(!out_file.is_open())
            std::cout<<"file open fail(write)" << std::endl;
    for(int i=0;i<2;++i)
    {
        for(int j=0;j<ori_data[0].size();++j)
            out_file << ori_data[i][j] << " " ;
        out_file << std::endl;

        for(int j=0;j<ori_data[0].size();++j)
            out_file << filted_num[i][j] << " " ;
        out_file << std::endl;
    }
    out_file.close();
    std::cout <<"DATA OPTIMIZED, and BE WRITTEN to file" << std::endl;
}