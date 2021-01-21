#include <iostream>
#include <cmath>

#include <fstream>
#include <sstream>

#include "../libs/display.h"
namespace plt = matplotlibcpp;
void plot_gridmap(const Dt_RECORD_EnvModelInfos &envModelInfo){
     std::vector<double> grid_x_dynamic;
     std::vector<double> grid_y_dynamic;
     std::vector<double> grid_x_static;
     std::vector<double> grid_y_static;
     for(uint32 x=0; x<400; ++x)
     for(uint32 y=0; y<200; ++y)
     {
         if(envModelInfo.ObstacleGridMap[x][y] == 1)
         {
             double x_vehi = 40 - static_cast<double>(x)/10;
             double y_vehi = 10 - static_cast<double>(y)/10; 

             grid_y_static.push_back(x_vehi);
             grid_x_static.push_back(y_vehi*-1);
         }
         if(envModelInfo.ObstacleGridMap[x][y] == 2)
         {
             double x_vehi = 40 -static_cast<double>(x)/10;
             double y_vehi = 10 -static_cast<double>(y)/10; 
             grid_y_dynamic.push_back(x_vehi);
             grid_x_dynamic.push_back(y_vehi*-1);

         }
     }
     plt::plot(grid_x_static,grid_y_static,"k.");// black point  -> static
     plt::plot(grid_x_dynamic,grid_y_dynamic,"c.");// 


}

void plot_lane(const hdMapTrajectory &TrajectoryPoints, const Dt_RECORD_HdmapFrontPLane &HdmapFrontPLane)
{
    std::vector<std::vector<double>> lane_left_x(TrajectoryPoints.pathLane[0].segNum); 
    std::vector<std::vector<double>> lane_left_y(TrajectoryPoints.pathLane[0].segNum); 
    std::vector<std::vector<double>> lane_right_x(TrajectoryPoints.pathLane[0].segNum); 
    std::vector<std::vector<double>> lane_right_y(TrajectoryPoints.pathLane[0].segNum); 
    std::vector<std::vector<double>> lane_safe_x(2);
    std::vector<std::vector<double>> lane_safe_y(2); 
    VehicleSize vehicle_size;
    double safe_lane_width = vehicle_size.width + 0.2;

    for(uint32 seg=0; seg < TrajectoryPoints.pathLane[0].segNum; ++seg)
    {
        double lanewidth_ = HdmapFrontPLane.PlanSeg[seg].Lane[0].lane_width;
        // DEBUG("lane width: %f \r\n",lanewidth_);
        for(uint32 node=0; node <TrajectoryPoints.pathLane[0].hdmapPathInfo[seg].laneInfos[0].nodeNum;++node)
        {
            double x_= TrajectoryPoints.pathLane[0].hdmapPathInfo[seg].laneInfos[0].laneNodeInfos[node].x;
            double y_= TrajectoryPoints.pathLane[0].hdmapPathInfo[seg].laneInfos[0].laneNodeInfos[node].y;
            double h_= TrajectoryPoints.pathLane[0].hdmapPathInfo[seg].laneInfos[0].laneNodeInfos[node].heading;
            
            // std::cout << x_ <<" " << x_ + -lanewidth_/2 * std::cos(h_/180 *M_PI) << " " <<lanewidth_/2 * std::cos(h_/180 *M_PI) << std::endl;
            // std::cout << y_ <<" " << (y_ - -lanewidth_/2 * std::sin(h_/180 *M_PI)) * -1 << " " <<lanewidth_/2 * std::cos(h_/180 *M_PI) <<" " << h_<< std::endl;

            lane_left_x[seg].push_back((y_ - -lanewidth_/2 * std::cos(h_/180 *M_PI)) * -1); 
            lane_left_y[seg].push_back((x_ + -lanewidth_/2 * std::sin(h_/180 *M_PI))); 
            lane_right_x[seg].push_back((y_ - lanewidth_/2 * std::cos(h_/180 *M_PI)) * -1); 
            lane_right_y[seg].push_back((x_ + lanewidth_/2 * std::sin(h_/180 *M_PI)));   
            lane_safe_x[0].push_back((y_ - -safe_lane_width/2 * std::cos(h_/180 *M_PI)) * -1);
            lane_safe_y[0].push_back(x_ + -safe_lane_width/2 * std::sin(h_/180 *M_PI));
            lane_safe_x[1].push_back((y_ - safe_lane_width/2 * std::cos(h_/180 *M_PI)) * -1);
            lane_safe_y[1].push_back(x_ + safe_lane_width/2 * std::sin(h_/180 *M_PI));
        }
    }
    for(uint32 seg=0; seg < TrajectoryPoints.pathLane[0].segNum; ++seg)
    {
        plt::plot(lane_left_x[seg], lane_left_y[seg],"r");
        plt::plot(lane_right_x[seg], lane_right_y[seg],"r");
    }
    plt::plot(lane_safe_x[0], lane_safe_y[0],"y");
    plt::plot(lane_safe_x[1], lane_safe_y[1],"y");
    

}
void plot_box(std::vector<double> cent_x, std::vector<double> cent_y, std::vector<double>heading, std::vector<int> type, const enum ObsType &obs_type)
{
    // cent_x= {0, 10, 23, 7 };
    // cent_y= {0, 1,  -1, 2 };
    // heading={0, 10, -10, 9};

    std::vector<std::vector<double>> box_xs(cent_x.size()); // number of box
    std::vector<std::vector<double>> box_ys(cent_y.size());
    

    for(uint32 idx=0; idx < cent_x.size(); ++idx)
    {
        std::vector<std::vector<double>> corners;
        if(cent_x.size()==0) 
        {
            std::cout << "plot_box: no input" << std::endl;
            return;
        }
       
        if(type[idx] ==1 )
        {
            Box2d box({cent_x[idx], cent_y[idx]}, heading[idx], 4.68, 1.93); // vehicle
            box.GetAllCorners(&corners);
        }
            
        else
        {
            Box2d box({cent_x[idx], cent_y[idx]}, heading[idx], 1, 1); // pedestrian
            box.GetAllCorners(&corners);
        }
       // return;
        
       corners.push_back(corners[0]); // pad first point to plot a whole box
        
        for(uint32 c_idx=0; c_idx < corners.size();++c_idx)
        {
            // box_xs[idx].push_back(corners[c_idx][1] *-1);
            // box_ys[idx].push_back(corners[c_idx][0]);
            box_xs[idx].push_back(corners[c_idx][0]);
            box_ys[idx].push_back(corners[c_idx][1]);
        }
        
    }
    // std::cout << "cent_X: " <<cent_x.size()<<std::endl;
    for(uint32 idx=0; idx < cent_x.size(); ++idx)
    {
        // std::cout << "x0: " << box_xs[idx][0] <<"y0: "<<box_ys[idx][0]<<std::endl;
        // std::cout << "x1: " << box_xs[idx][1] <<"y1: "<<box_ys[idx][1]<<std::endl;
        // std::cout << "x2: " << box_xs[idx][2] <<"y2: "<<box_ys[idx][2]<<std::endl;
        // std::cout << "x3: " << box_xs[idx][3] <<"y3: "<<box_ys[idx][3]<<std::endl;
        if(obs_type == ObsType::Ego_Vehicle)
            plt::plot(box_xs[idx],box_ys[idx],"c-");
        if(obs_type == ObsType::CIPV_V1)
            plt::plot(box_xs[idx],box_ys[idx],"b-");

        if(obs_type == ObsType::CIPV_V2)
            plt::plot(box_xs[idx],box_ys[idx],"b--");

        if(obs_type == ObsType::NotImpor_Obs)
            plt::plot(box_xs[idx],box_ys[idx],"b--");

        
    }

}


void plot_vehicleCoordi_wind(const hdMapTrajectory &hdMapTrajectory,const Dt_RECORD_HdmapFrontPLane &HdmapFrontPLane, const Dt_RECORD_EnvModelInfos &envModelInfo, const objSecList &selectObj){
    std::vector<double> ref_path_x;
    std::vector<double> ref_path_y;
    std::vector<double> ref_path_h;

    std::vector<double> dis_cipv_1_x;
    std::vector<double> dis_cipv_1_y;
    std::vector<double> dis_cipv_1_h;
    std::vector<int> dis_cipv_1_type;

    std::vector<double> dis_cipv_2_x;
    std::vector<double> dis_cipv_2_y;
    std::vector<double> dis_cipv_2_h;
    std::vector<int> dis_cipv_2_type;

    std::vector<double> obj_list_cent_x;
    std::vector<double> obj_list_cent_y;
    std::vector<double> obj_list_heading;
    std::vector<int> obj_list_type;
    
    plt::subplot2grid(2,3,0,0,2,1);
    plt::xlim(-10,10);
    plt::ylim(-10,40);

    for(unsigned int seg_idx=0; seg_idx< hdMapTrajectory.pathLane[0].segNum; seg_idx++)
    for(unsigned int node_idx=0; node_idx< hdMapTrajectory.pathLane[0].hdmapPathInfo[seg_idx].laneInfos[0].nodeNum; node_idx+=10)
    {
         ref_path_x.push_back(hdMapTrajectory.pathLane[0].hdmapPathInfo[seg_idx].laneInfos[0].laneNodeInfos[node_idx].y  * -1);
         ref_path_y.push_back(hdMapTrajectory.pathLane[0].hdmapPathInfo[seg_idx].laneInfos[0].laneNodeInfos[node_idx].x);
         ref_path_h.push_back(hdMapTrajectory.pathLane[0].hdmapPathInfo[seg_idx].laneInfos[0].laneNodeInfos[node_idx].heading);
    }
    if(selectObj.frontMid.postion ==1 )// CIPV_1
    {
        dis_cipv_1_x.push_back(selectObj.frontMid.obj.pos_y * -1);
        dis_cipv_1_y.push_back(selectObj.frontMid.obj.pos_x);
        dis_cipv_1_h.push_back(selectObj.frontMid.obj.heading);
        dis_cipv_1_type.push_back(selectObj.frontMid.obj.type);

    }
    if(selectObj.frontMidLeft.postion ==1 ) // CIPV_2
    {
        dis_cipv_2_x.push_back(selectObj.frontMidLeft.obj.pos_y * -1);
        dis_cipv_2_y.push_back(selectObj.frontMidLeft.obj.pos_x);
        dis_cipv_2_h.push_back(selectObj.frontMidLeft.obj.heading);
        dis_cipv_2_type.push_back((int)selectObj.frontMidLeft.obj.type);
    }

    if( selectObj.frontMidRight.postion ==1) // CIPV_2
    {
        dis_cipv_2_x.push_back(selectObj.frontMidRight.obj.pos_y * -1);
        dis_cipv_2_y.push_back(selectObj.frontMidRight.obj.pos_x);
        dis_cipv_2_h.push_back(selectObj.frontMidRight.obj.heading);
        dis_cipv_2_type.push_back((int)selectObj.frontMidRight.obj.type);
    }

    if(selectObj.frontLeft.postion ==1 ) // CIPV_2
    {
        dis_cipv_2_x.push_back(selectObj.frontLeft.obj.pos_y * -1);
        dis_cipv_2_y.push_back(selectObj.frontLeft.obj.pos_x);
        dis_cipv_2_h.push_back(selectObj.frontLeft.obj.heading);
        dis_cipv_2_type.push_back((int)selectObj.frontLeft.obj.type);
    }

    if( selectObj.frontRight.postion ==1) // CIPV_2
    {
        dis_cipv_2_x.push_back(selectObj.frontRight.obj.pos_y * -1);
        dis_cipv_2_y.push_back(selectObj.frontRight.obj.pos_x);
        dis_cipv_2_h.push_back(selectObj.frontRight.obj.heading);
        dis_cipv_2_type.push_back((int)selectObj.frontRight.obj.type);
    }
    
      

    // std::cout << "obs_num: "<<  envModelInfo.obstacle_num << std::endl;
    for(unsigned int obs_idx=0; obs_idx < envModelInfo.obstacle_num; ++obs_idx)
    {
        obj_list_cent_x.push_back(envModelInfo.Obstacles[obs_idx].pos_y * -1);
        obj_list_cent_y.push_back(envModelInfo.Obstacles[obs_idx].pos_x);
        obj_list_heading.push_back(envModelInfo.Obstacles[obs_idx].heading);
        obj_list_type.push_back(envModelInfo.Obstacles[obs_idx].type);

        // std::cout << envModelInfo.Obstacles->pos_x<< envModelInfo.Obstacles->pos_y << std::endl;
    }
   
    plt::plot(ref_path_x, ref_path_y,"y--");
    plt::plot(obj_list_cent_x, obj_list_cent_y,"bo");
    plt::plot(dis_cipv_1_x, dis_cipv_1_y,"ro");
    plt::plot(dis_cipv_2_x, dis_cipv_2_y,"ro");

    plot_lane(hdMapTrajectory, HdmapFrontPLane);
    plot_box(std::vector<double>{0},std::vector<double>{0},std::vector<double>{0}, std::vector<int>{1},ObsType::Ego_Vehicle);
    // plot_box(obj_list_cent_x,obj_list_cent_y,obj_list_heading, obj_list_type, ObsType::NotImpor_Obs);
    // plot_box(dis_cipv_1_x,dis_cipv_1_y,dis_cipv_1_h, dis_cipv_1_type, ObsType::CIPV_V1);
    // plot_box(dis_cipv_2_x,dis_cipv_2_y,dis_cipv_2_h, dis_cipv_2_type, ObsType::CIPV_V2);

    // plot_gridmap(envModelInfo);
    // plot_box(obj_list_cent_x,obj_list_cent_y,obj_list_heading, ObsType::NotImpor_Obs);
}

void plot_infoList_wind(const Dt_RECORD_HdmapInfo &hdmapInfos, const Dt_RECORD_AccInfo &vehicleInfo){
    plt::subplot2grid(2,3,0,2,2,1);
    plt::xlim(0,20);
    plt::ylim(0,20); 
    std::vector<string> DisMode{"Replay" , "Vehicle"};
    plt::text(1,18, "Vehicle / Replay: " + DisMode[0]);
    plt::text(1,16, "On Path: " + std::to_string(hdmapInfos.planpath));
    plt::text(1,14, "Vehicle Speed (KPH): " + std::to_string(vehicleInfo.ESP_VehSpd * 3.6));
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

int plot(const hdMapTrajectory &hdMapTrajectory, const Dt_RECORD_HdmapFrontPLane &HdmapFrontPLane, const Dt_RECORD_HdmapInfo &hdmapInfos, const Dt_RECORD_HdmapFrontPLane &globePLane,\
    const Dt_RECORD_LocalizationInfo &localInfos, const Dt_RECORD_EnvModelInfos &envModelInfo, const Dt_RECORD_AccInfo &vehicleInfo, const objSecList &selectObj) {
    plt::ion();
    plot_vehicleCoordi_wind(hdMapTrajectory, HdmapFrontPLane, envModelInfo, selectObj);
    plot_globalCoordi_wind(localInfos, hdmapInfos);
    plot_mapCoordi_wind(localInfos, globePLane);
    plot_infoList_wind(hdmapInfos, vehicleInfo);
    plt::pause(0.1);
    plt::clf();
}
