#include<iostream>
#include<vector>
#include<fstream>
#include<sstream>
#include<memory>
#include "../LIB/median_filter.h"
#include "../LIB/types.pb.h"

// #define VIRTUAL_DATA_RANDOM
// #define REAL_DATA_SET
// #define REAL_DATA_SELECT
#define REAL_PROTUBUF_DATA_SELECT
std::vector<std::vector<double>> Solution::data_windows(20); //apply Memory for windows 

/*
@breaf taget selection base on front rectangle
@param obstacle_list: original obstacle list
@param selected_obj_list: the selected obstacle which affects the ego car most
*/
void obstacle_select(const std::vector<obj_sel> &obstacle_list, objSecList *selected_obj_list){
    std::vector<obj_sel> obj_in_field;
    int tar_obs_idx=-1;
    double _min_X =999;

    for(int i=0; i<obstacle_list.size();++i)
    {
        if(obstacle_list[i].pos_x > 0.5 && obstacle_list[i].pos_x < 30 && \
            obstacle_list[i].pos_y > -1.7 && obstacle_list[i].pos_y < 1.7)
        obj_in_field.push_back(obstacle_list[i]);

        for(int i=0; i<obj_in_field.size(); ++i)
        {
            if(obj_in_field[i].pos_x < _min_X)
            {
                _min_X = obj_in_field[i].pos_x;
                tar_obs_idx = i;
            }
        }
        selected_obj_list->frontMid.obj.id = obj_in_field[tar_obs_idx].id;
        selected_obj_list->frontMid.obj.pos_x = obj_in_field[tar_obs_idx].pos_x;
        selected_obj_list->frontMid.obj.pos_y = obj_in_field[tar_obs_idx].pos_y;
    }
   
}

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
@breaf:
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



//* USE VIRTUAL DATA GENERATED*
# ifdef VIRTUAL_DATA_RANDOM
{
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
}
#endif

//* USE REAL FILE DATA SETTED*
#ifdef REAL_DATA_SET
{
    std::string line_str;
    std::string word_str;
    in_file.open("../log/data_convert",std::ios::in);
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
}
#endif


//* USE REAL FILE DATA SETECTED*
#ifdef REAL_DATA_SELECT
    std::string line_str;
    std::string word_str;
    std::vector<objFrame> frame_list;

    in_file.open("../log/obstacle_list_info",std::ios::in);
    if(!in_file.is_open())
    {
        std::cout << "file open fail(read)" << std::endl;
    }

    while(!in_file.eof())  //read file to end
    {
        if(!std::getline(in_file, line_str))
        {
            std::cout << "ERROR: empty file <obstacle_list_inf0o>" << std::endl;
            return -1; 
        }
        
        while(1)
        {
            std::stringstream linestream(line_str);
            linestream >> word_str;
            if(word_str == "Current_Frame:")
            {
                linestream >> word_str;
                std::shared_ptr<objFrame> obj_frame = std::make_shared<objFrame>();
                obj_frame->frame_num = stoi(word_str);
                // std::cout <<"frame_num: "<< obj_frame->frame_num <<std::endl;
                while(1)
                {
                    if(!std::getline(in_file,line_str)) break;
                    if(line_str.find("Current_Frame")!=-1)
                    {
                        frame_list.push_back(*obj_frame);
                        break;
                    }
                    else
                    {
                        std::shared_ptr<obj_sel> obj_ptr = std::make_shared<obj_sel>(); // new Obstacle

                        obj_ptr->id = stoi(line_str);
                        // std::cout << "id: "  << obj_ptr->id << std::endl;
                        std::getline(in_file,line_str);
                        obj_ptr->pos_x = stod(line_str);
                        // std::cout << "pos_x: "  << obj_ptr->pos_x << std::endl;
                        std::getline(in_file,line_str);
                        obj_ptr->pos_y = stod(line_str);
                        // std::cout << "pos_y: "  << obj_ptr->pos_y << std::endl;
                        obj_frame->obj.push_back(*obj_ptr);  //save this obstacle
                    }
                }
            }
            if(in_file.eof()) break;
        }
    }
    in_file.close();


    //*** OBJECT SELECTION ***//
    std::vector<objSecList> selected_obj_list_;
    objSecList  selected_obj_list;
    for(int i =0; i < frame_list.size(); ++i){
        obstacle_select(frame_list[i].obj, &selected_obj_list);
        selected_obj_list_.push_back(selected_obj_list);
    }
   
    for(int i=0;i<selected_obj_list_.size();++i)
    {
        ori_data[0].push_back( selected_obj_list_[i].frontMid.obj.pos_x );
        ori_data[1].push_back( selected_obj_list_[i].frontMid.obj.pos_y );
        std::cout << "frontMid.obj.id " << selected_obj_list_[i].frontMid.obj.id << std::endl; 
        // std::cout << "frontMid.obj.pos_x " << selected_obj_list_[i].frontMid.obj.pos_x << std::endl; 
        // std::cout << "frontMid.obj.pos_y " << selected_obj_list_[i].frontMid.obj.pos_y << std::endl; 
    }
#endif

//* USE REAL PROTOBUF FILE DATA SETECTED*
#ifdef REAL_PROTUBUF_DATA_SELECT
    std::vector<objFrame> frame_list;
    
    in_file.open("../log/obstacle_list_info",std::ios::in);
    if(!in_file.is_open())
    {
        std::cout << "file open fail(read)" << std::endl;
    }

    while(!in_file.eof())  //read file to end
    {
        auto logfile_string_pb = in_file.read();
    }
    in_file.close();
    


    //*** OBJECT SELECTION ***//
    std::vector<objSecList> selected_obj_list_;
    objSecList  selected_obj_list;
    for(int i =0; i < frame_list.size(); ++i){
        obstacle_select(frame_list[i].obj, &selected_obj_list);
        selected_obj_list_.push_back(selected_obj_list);
    }
   
    for(int i=0;i<selected_obj_list_.size();++i)
    {
        ori_data[0].push_back( selected_obj_list_[i].frontMid.obj.pos_x );
        ori_data[1].push_back( selected_obj_list_[i].frontMid.obj.pos_y );
        std::cout << "frontMid.obj.id " << selected_obj_list_[i].frontMid.obj.id << std::endl; 
        // std::cout << "frontMid.obj.pos_x " << selected_obj_list_[i].frontMid.obj.pos_x << std::endl; 
        // std::cout << "frontMid.obj.pos_y " << selected_obj_list_[i].frontMid.obj.pos_y << std::endl; 
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
    out_file.open("../log/data_convert",std::ios::out);
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
    std::cout <<"[median_filter.cc]: DATA OPTIMIZED SUCCESS, and data was WRITTEN to file" << std::endl;
}