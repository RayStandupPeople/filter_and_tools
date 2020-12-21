#include <iostream>
#include <cmath>

#include <fstream>
#include <sstream>

#include "../libs/display.h"

namespace plt = matplotlibcpp;

void plot_vehicleCoordi_wind(const hdMapTrajectory &Trajectory, const Dt_RECORD_EnvModelInfos &envModelInfo, const objSecList &selectObj){
    std::vector<double> ref_path_x;
    std::vector<double> ref_path_y;
    std::vector<double> ref_path_h;
    std::vector<double> dis_cipv_x;
    std::vector<double> dis_cipv_y;
    std::vector<double> obj_list_x;
    std::vector<double> obj_list_y;



    plt::subplot2grid(2,3,0,0,2,1);
    plt::xlim(-10,10);
    plt::ylim(-10,40);
    for(unsigned int lane_seg_idx=0; lane_seg_idx < Trajectory.pathLane[0].segNum; ++lane_seg_idx)
    for(unsigned int node_idx=0; node_idx<  Trajectory.pathLane[0].hdmapPathInfo[lane_seg_idx].laneInfos[0].nodeNum; node_idx+=5)
    {
         ref_path_x.push_back(Trajectory.pathLane[0].hdmapPathInfo[lane_seg_idx].laneInfos[0].laneNodeInfos[node_idx].y * -1);
         ref_path_y.push_back(Trajectory.pathLane[0].hdmapPathInfo[lane_seg_idx].laneInfos[0].laneNodeInfos[node_idx].x);
         ref_path_h.push_back(Trajectory.pathLane[0].hdmapPathInfo[lane_seg_idx].laneInfos[0].laneNodeInfos[node_idx].heading);
    }
    dis_cipv_x.push_back(selectObj.frontMid.obj.pos_y * -1);
    dis_cipv_y.push_back(selectObj.frontMid.obj.pos_x);
    // std::cout << "obs_num: "<<  envModelInfo.obstacle_num << std::endl;
    for(unsigned int obs_idx=0; obs_idx < envModelInfo.obstacle_num; ++obs_idx)
    {
        obj_list_x.push_back(envModelInfo.Obstacles->pos_y * -1);
        obj_list_y.push_back(envModelInfo.Obstacles->pos_x);
        // std::cout << envModelInfo.Obstacles->pos_x<< envModelInfo.Obstacles->pos_y << std::endl;
    }

    plt::plot(ref_path_x, ref_path_y,"y--");
    plt::plot(dis_cipv_x, dis_cipv_y,"ro");
    plt::plot(obj_list_x, obj_list_y,"bo");

    
}

void plot_infoList_wind(const Dt_RECORD_HdmapInfo &hdmapInfos){
    plt::subplot2grid(2,3,0,2,2,1);
    plt::xlim(0,20);
    plt::ylim(0,20); 
    std::vector<string> DisMode{"Replay" , "Vehicle"};
    plt::text(1,18, "Vehicle / Replay: " + DisMode[0]);
    plt::text(1,16, "On Path: " + std::to_string(hdmapInfos.planpath));
    plt::text(1,14, "Vehicle Speed (KPH): " + std::to_string(3.6));
    plt::text(1,12, "Steer Wheel Ang: " + std::to_string(120));
    plt::text(1,10, "Sts Command: " + std::to_string(7));
    plt::text(1,8, "Decision Command: " + std::to_string(1));

}
void plot_mapCoordi_wind(const Dt_RECORD_LocalizationInfo &localInfos, const Dt_RECORD_HdmapFrontPLane &globePLane){
    plt::subplot2grid(2,3,0,1,1,1);
    std::vector<std::vector<double>> path_seg_X;
    std::vector<std::vector<double>> path_seg_Y;
    std::vector<std::vector<double>> path_seg_H;
    std::vector<double>loc_X;
    std::vector<double>loc_Y;
    loc_X.push_back(localInfos.LocalizationResult.x);
    loc_Y.push_back(localInfos.LocalizationResult.y);

    for(unsigned int lane_seg_idx=0; lane_seg_idx < globePLane.plan_seg_count; ++lane_seg_idx)
    {
        std::vector<double> _x;
        std::vector<double> _y;
        std::vector<double> _heading;
        for(unsigned int node_idx=0; node_idx<globePLane.PlanSeg[lane_seg_idx].Lane[0].node_count; node_idx+=5)
        {
           
            _x.push_back(globePLane.PlanSeg[lane_seg_idx].Lane[0].LaneNode[node_idx].hdmap_y * -1);
            _y.push_back(globePLane.PlanSeg[lane_seg_idx].Lane[0].LaneNode[node_idx].hdmap_x);
            _heading.push_back(globePLane.PlanSeg[lane_seg_idx].Lane[0].LaneNode[node_idx].heading);
        }
        path_seg_X.push_back(_x);
        path_seg_Y.push_back(_y);
        path_seg_H.push_back(_heading);

    }

    for(unsigned int lane_seg_idx=0; lane_seg_idx < globePLane.plan_seg_count; ++lane_seg_idx)
        plt::plot(path_seg_X[lane_seg_idx], path_seg_Y[lane_seg_idx],"y--");
    plt::plot(loc_X, loc_Y,"ro");

}
void plot_globalCoordi_wind(const Dt_RECORD_LocalizationInfo &localInfos, const Dt_RECORD_HdmapInfo &hdmapInfos){
    plt::subplot2grid(2,3,1,1,1,1);

    std::vector<double>global_path_x_lg ={-120, -30, -20, -15, -10, -10, -13, -30, -115};
    std::vector<double>global_path_y_lg ={-30, -30, -28, -25, -20, 0, 6, 10, 10};

    std::vector<double>global_path_x_gwh ={0,   40,  60,  65,  70, 75,   80,  95, 106};
    std::vector<double>global_path_y_gwh ={105, 130, 140, 135, 125, 115, 110, 90, 71};
    std::vector<double>loc_X;
    std::vector<double>loc_Y;
    std::vector<double>start_X;
    std::vector<double>start_Y;
    std::vector<double>end_X;
    std::vector<double>end_Y;

    loc_X.push_back(localInfos.LocalizationResult.x);
    loc_Y.push_back(localInfos.LocalizationResult.y);
    if(loc_X[0]>0 && loc_Y[0]>0)
    {
        plt::xlim(-10,110);
        plt::ylim(60,150); 
        plt::plot(global_path_x_gwh, global_path_y_gwh,"b");
       
    }
    else
    {
        plt::xlim(-140,10);
        plt::ylim(-50,30);
        plt::plot(global_path_x_lg, global_path_y_lg,"b");
    }
    start_X.push_back(hdmapInfos.origin_x);
    start_Y.push_back(hdmapInfos.origin_y);
    end_X.push_back(hdmapInfos.goal_x);
    end_Y.push_back(hdmapInfos.goal_y);
    
    plt::plot(loc_X, loc_Y,"ro");
    plt::plot(start_X, start_Y,"or");
    plt::plot(end_X, end_Y,"or");
    plt::text(start_X[0]+1,start_Y[0]+1,"Start");
    plt::text(end_X[0]+1,end_Y[0]+1,"End");

}

int plot(const hdMapTrajectory &Trajectory, const Dt_RECORD_HdmapInfo &hdmapInfos, const Dt_RECORD_HdmapFrontPLane &globePLane, \
    const Dt_RECORD_LocalizationInfo &localInfos, const Dt_RECORD_EnvModelInfos &envModelInfo, const objSecList &selectObj) {
    
       plot_vehicleCoordi_wind(Trajectory, envModelInfo, selectObj);
       plot_globalCoordi_wind(localInfos, hdmapInfos);
       plot_mapCoordi_wind(localInfos, globePLane);
       plot_infoList_wind(hdmapInfos);
       plt::pause(0.1);
       plt::clf();
}