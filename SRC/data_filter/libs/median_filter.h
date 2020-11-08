#pragma once
#include "../../common/libs/types.h"
#include "../../common/libs/types.pb.h"


class MedianFilter{
    private:
        static std::vector<std::vector<double>> data_windows;
        void move_window_filter(double &data,  const char &filter_type, const size_t &wind_size, char window_idx);
    public:
        void data_stabilization(objSecList *selected_obj_list, const char &filter_type, const size_t &wind_size);
        void obstacle_select(const std::vector<pb_types::Obstacle> &obstacle, objSecList *selected_obj_list);
};