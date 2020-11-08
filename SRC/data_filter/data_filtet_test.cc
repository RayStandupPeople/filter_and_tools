#include <iostream>
#include <fstream>
#include <sstream>
#include "libs/median_filter.h"
#include "../common/libs/types.pb.h"
#include "../common/libs/types.h"

// #define VIRTUAL_DATA_RANDOM
// #define REAL_DATA_SET
// #define REAL_DATA_SELECT
#define REAL_PROTUBUF_DATA_SELECT

int main(int argc, char* argv[]){
    MedianFilter mf; // new class
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
        
        in_file.open("../../../log/wholeFile_info_pb",std::ios::in);
        if(!in_file.is_open())
        {
            std::cout << "file open fail(read)" << std::endl;
            return -1;
        }

        std::stringstream buffer;
        std::string file_str;
        buffer << in_file.rdbuf();
        file_str = buffer.str();
        in_file.close();

        
        pb_types::LogFile logfile_pb;
        logfile_pb.ParseFromString(file_str);

        std::cout << logfile_pb.frame_num();

        //*** OBJECT SELECTION ***//
        std::vector<objSecList> selected_obj_list_;
        objSecList  selected_obj_list;

        for(int i =0; i < logfile_pb.frame_size(); ++i)
        {
            std::vector<pb_types::Obstacle> pb_obstacle_vec;
            for(int j=0;j<logfile_pb.frame(i).obstacle_size();++j)
               pb_obstacle_vec.push_back(logfile_pb.frame(i).obstacle(j));

            mf.obstacle_select(pb_obstacle_vec, &selected_obj_list);
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
        mf.data_stabilization(&obj_sel_list,0,10); // ** KEY FUN **
        filted_num[0].push_back(obj_sel_list.frontMid.obj.pos_x);
        filted_num[1].push_back(obj_sel_list.frontMid.obj.pos_y);
    }

    
    /* DATA OUTPUT*/
    out_file.open("data/data_convert",std::ios::out);
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
    std::cout <<"[median_filter.cc]: DATA OPTIMIZED SUCCESS, and data was WRITTEN to file <data/data_convert>" << std::endl;
}