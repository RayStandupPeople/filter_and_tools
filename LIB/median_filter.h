#pragma once

struct obj_sel{
    int id;
    int type;
    double pos_x;
    double pos_y;
    double heading;
    double rel_speed_x;
    double rel_speed_y;
};

struct objSec
{
    int position;
    obj_sel obj;
};

struct objSecList
{
    objSec frontMid;
    objSec frontMidLeft;
    objSec frontMidRight;
    objSec objSecAEB;
};


struct objFrame
{
    int frame_num;
    std::vector<obj_sel> obj;
};

class Solution{
    private:
        static std::vector<std::vector<double>> data_windows;
        void move_window_filter(double &data,  const char &filter_type, const size_t &wind_size, char window_idx);
    public:
        void data_stabilization(objSecList *selected_obj_list, const char &filter_type, const size_t &wind_size);
        void obstacle_select(const std::vector<obj_sel> &obstacle_list, objSecList *selected_obj_list);
};