#pragma once

#include "types.h"
class Solution{
    private:
        static std::vector<std::vector<double>> data_windows;
        void move_window_filter(double &data,  const char &filter_type, const size_t &wind_size, char window_idx);
    public:
        void data_stabilization(objSecList *selected_obj_list, const char &filter_type, const size_t &wind_size);
        void obstacle_select(const std::vector<obj_sel> &obstacle_list, objSecList *selected_obj_list);
};