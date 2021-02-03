#include <iostream>
#include <cmath>

#include <fstream>
#include <sstream>
#include <thread>
#include "../libs/display.h"
namespace plt = matplotlibcpp;
void plot_gridmap(const Dt_RECORD_EnvModelInfos &envModelInfo, PlotItems &plot_items){
     std::vector<double> grid_x_dynamic;
     std::vector<double> grid_y_dynamic;
     std::vector<double> grid_x_static;
     std::vector<double> grid_y_static;
     for(uint32 x=0; x<400; x+=2) //grid scatter n times
     for(uint32 y=0; y<200; y+=2)  // left and right each display 5m
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

     plot_items.grid[0].x.assign(grid_x_static.begin(), grid_x_static.end());
     plot_items.grid[0].y.assign(grid_y_static.begin(), grid_y_static.end());
     plot_items.grid[1].x.assign(grid_x_dynamic.begin(), grid_x_dynamic.end());
     plot_items.grid[1].y.assign(grid_y_dynamic.begin(), grid_y_dynamic.end());


}


void plot_lane(const hdMapTrajectory &TrajectoryPoints, const DecisionToPC &rev_DecisionToPC_data, PlotItems &plot_items)
{

    std::vector<std::vector<double>> lane_left_x(TrajectoryPoints.pathLane[0].segNum); 
    std::vector<std::vector<double>> lane_left_y(TrajectoryPoints.pathLane[0].segNum); 
    std::vector<std::vector<double>> lane_right_x(TrajectoryPoints.pathLane[0].segNum); 
    std::vector<std::vector<double>> lane_right_y(TrajectoryPoints.pathLane[0].segNum); 
    std::vector<std::vector<double>> lane_lleft_x(TrajectoryPoints.pathLane[0].segNum); 
    std::vector<std::vector<double>> lane_lleft_y(TrajectoryPoints.pathLane[0].segNum); 
    std::vector<std::vector<double>> lane_rright_x(TrajectoryPoints.pathLane[0].segNum); 
    std::vector<std::vector<double>> lane_rright_y(TrajectoryPoints.pathLane[0].segNum); 
    std::vector<std::vector<double>> lane_safe_x(2);
    std::vector<std::vector<double>> lane_safe_y(2); 
    VehicleSize vehicle_size;
    Dt_RECORD_HdmapFrontPLane HdmapFrontPLane = rev_DecisionToPC_data.my_hdmapFrontPLaneInfo;
    Dt_RECORD_HdmapLocalLane  hdmapLocalLaneInfo = rev_DecisionToPC_data.my_hdmapLocalLaneInfo; 
    Dt_RECORD_HdmapInfo my_hdmapInfo = rev_DecisionToPC_data.my_hdmapInfo;
    double safe_lane_width = vehicle_size.width + 0.2;

    for(uint32 seg=0; seg < TrajectoryPoints.pathLane[0].segNum; ++seg)
    {
        double lanewidth_ = HdmapFrontPLane.PlanSeg[seg].Lane[0].lane_width;
        // DEBUG("lane width: %f \r\n",lanewidth_);
        for(uint32 node=0; node <TrajectoryPoints.pathLane[0].hdmapPathInfo[seg].laneInfos[0].nodeNum;node+=3) // scatter 3 times
        {
            double x_= TrajectoryPoints.pathLane[0].hdmapPathInfo[seg].laneInfos[0].laneNodeInfos[node].x;
            double y_= TrajectoryPoints.pathLane[0].hdmapPathInfo[seg].laneInfos[0].laneNodeInfos[node].y;
            double h_= TrajectoryPoints.pathLane[0].hdmapPathInfo[seg].laneInfos[0].laneNodeInfos[node].heading;
            
            // std::cout << x_ <<" " << x_ + -lanewidth_/2 * std::cos(h_/180 *M_PI) << " " <<lanewidth_/2 * std::cos(h_/180 *M_PI) << std::endl;
            // std::cout << y_ <<" " << (y_ - -lanewidth_/2 * std::sin(h_/180 *M_PI)) * -1 << " " <<lanewidth_/2 * std::cos(h_/180 *M_PI) <<" " << h_<< std::endl;
            int lane_num;
            if(my_hdmapInfo.planpath)
                lane_num = TrajectoryPoints.pathLane[0].hdmapPathInfo[seg].laneNum;
            else
                lane_num = TrajectoryPoints.localPath[0].hdmapPathInfo[seg].laneNum;
            switch(lane_num)
            {
                case 0: //crossing
                    lane_safe_x[0].push_back((y_ - -safe_lane_width/2 * std::cos(h_/180 *M_PI)) * -1);
                    lane_safe_y[0].push_back(x_ + -safe_lane_width/2 * std::sin(h_/180 *M_PI));
                    lane_safe_x[1].push_back((y_ - safe_lane_width/2 * std::cos(h_/180 *M_PI)) * -1);
                    lane_safe_y[1].push_back(x_ + safe_lane_width/2 * std::sin(h_/180 *M_PI));
                    break;
                case 1: //only ego lane
                    lane_left_x[seg].push_back((y_ - -lanewidth_/2 * std::cos(h_/180 *M_PI)) * -1); 
                    lane_left_y[seg].push_back((x_ + -lanewidth_/2 * std::sin(h_/180 *M_PI))); 
                    lane_right_x[seg].push_back((y_ - lanewidth_/2 * std::cos(h_/180 *M_PI)) * -1); 
                    lane_right_y[seg].push_back((x_ + lanewidth_/2 * std::sin(h_/180 *M_PI)));   
                    lane_safe_x[0].push_back((y_ - -safe_lane_width/2 * std::cos(h_/180 *M_PI)) * -1);
                    lane_safe_y[0].push_back(x_ + -safe_lane_width/2 * std::sin(h_/180 *M_PI));
                    lane_safe_x[1].push_back((y_ - safe_lane_width/2 * std::cos(h_/180 *M_PI)) * -1);
                    lane_safe_y[1].push_back(x_ + safe_lane_width/2 * std::sin(h_/180 *M_PI));
                    break;
                case 2: //ego and left lane
                    lane_lleft_x[seg].push_back((y_ - lanewidth_/2*3 * std::cos(h_/180 *M_PI)) * -1); 
                    lane_lleft_y[seg].push_back((x_ + lanewidth_/2*3 * std::sin(h_/180 *M_PI))); 
                    lane_left_x[seg].push_back((y_ - -lanewidth_/2 * std::cos(h_/180 *M_PI)) * -1); 
                    lane_left_y[seg].push_back((x_ + -lanewidth_/2 * std::sin(h_/180 *M_PI))); 
                    lane_right_x[seg].push_back((y_ - lanewidth_/2 * std::cos(h_/180 *M_PI)) * -1); 
                    lane_right_y[seg].push_back((x_ + lanewidth_/2 * std::sin(h_/180 *M_PI)));   
                    lane_safe_x[0].push_back((y_ - -safe_lane_width/2 * std::cos(h_/180 *M_PI)) * -1);
                    lane_safe_y[0].push_back(x_ + -safe_lane_width/2 * std::sin(h_/180 *M_PI));
                    lane_safe_x[1].push_back((y_ - safe_lane_width/2 * std::cos(h_/180 *M_PI)) * -1);
                    lane_safe_y[1].push_back(x_ + safe_lane_width/2 * std::sin(h_/180 *M_PI));
                    break;
                case 3: //ego and right lane
                    lane_rright_x[seg].push_back((y_ - -lanewidth_/2*3 * std::cos(h_/180 *M_PI)) * -1); 
                    lane_rright_y[seg].push_back((x_ + -lanewidth_/2*3 * std::sin(h_/180 *M_PI)));   
                    lane_left_x[seg].push_back((y_ - -lanewidth_/2 * std::cos(h_/180 *M_PI)) * -1); 
                    lane_left_y[seg].push_back((x_ + -lanewidth_/2 * std::sin(h_/180 *M_PI))); 
                    lane_right_x[seg].push_back((y_ - lanewidth_/2 * std::cos(h_/180 *M_PI)) * -1); 
                    lane_right_y[seg].push_back((x_ + lanewidth_/2 * std::sin(h_/180 *M_PI)));   
                    lane_safe_x[0].push_back((y_ - -safe_lane_width/2 * std::cos(h_/180 *M_PI)) * -1);
                    lane_safe_y[0].push_back(x_ + -safe_lane_width/2 * std::sin(h_/180 *M_PI));
                    lane_safe_x[1].push_back((y_ - safe_lane_width/2 * std::cos(h_/180 *M_PI)) * -1);
                    lane_safe_y[1].push_back(x_ + safe_lane_width/2 * std::sin(h_/180 *M_PI));
                    break;
                case 4: // both lane and ego
                    lane_lleft_x[seg].push_back((y_ - lanewidth_/2*3 * std::cos(h_/180 *M_PI)) * -1); 
                    lane_lleft_y[seg].push_back((x_ + lanewidth_/2*3 * std::sin(h_/180 *M_PI))); 
                    lane_rright_x[seg].push_back((y_ - -lanewidth_/2*3 * std::cos(h_/180 *M_PI)) * -1); 
                    lane_rright_y[seg].push_back((x_ + -lanewidth_/2*3 * std::sin(h_/180 *M_PI)));   
                    lane_left_x[seg].push_back((y_ - -lanewidth_/2 * std::cos(h_/180 *M_PI)) * -1); 
                    lane_left_y[seg].push_back((x_ + -lanewidth_/2 * std::sin(h_/180 *M_PI))); 
                    lane_right_x[seg].push_back((y_ - lanewidth_/2 * std::cos(h_/180 *M_PI)) * -1); 
                    lane_right_y[seg].push_back((x_ + lanewidth_/2 * std::sin(h_/180 *M_PI)));   
                    lane_safe_x[0].push_back((y_ - -safe_lane_width/2 * std::cos(h_/180 *M_PI)) * -1);
                    lane_safe_y[0].push_back(x_ + -safe_lane_width/2 * std::sin(h_/180 *M_PI));
                    lane_safe_x[1].push_back((y_ - safe_lane_width/2 * std::cos(h_/180 *M_PI)) * -1);
                    lane_safe_y[1].push_back(x_ + safe_lane_width/2 * std::sin(h_/180 *M_PI));
                    break;
            }

        }
    }

    for(uint32 seg=0; seg < TrajectoryPoints.pathLane[0].segNum; ++seg)
    {
       
        // plot midLane
        plot_items.lane_midL[seg].x.assign(lane_left_x[seg].begin(), lane_left_x[seg].end());
        plot_items.lane_midL[seg].y.assign(lane_left_y[seg].begin(), lane_left_y[seg].end());
        plot_items.lane_midR[seg].x.assign(lane_right_x[seg].begin(),lane_right_x[seg].end());
        plot_items.lane_midR[seg].y.assign(lane_right_y[seg].begin(),lane_right_y[seg].end());

         // plot SideLane
        plot_items.lane_left[seg].x.assign(lane_lleft_x[seg].begin(), lane_lleft_x[seg].end());
        plot_items.lane_left[seg].y.assign(lane_lleft_y[seg].begin(), lane_lleft_y[seg].end());
        plot_items.lane_right[seg].x.assign(lane_rright_x[seg].begin(),lane_rright_x[seg].end());
        plot_items.lane_right[seg].y.assign(lane_rright_y[seg].begin(),lane_rright_y[seg].end());

    }
    // plot safeline
    plot_items.safeline[0].x.assign(lane_safe_x[0].begin(),lane_safe_x[0].end());
    plot_items.safeline[0].y.assign(lane_safe_y[0].begin(),lane_safe_y[0].end());
    plot_items.safeline[1].x.assign(lane_safe_x[1].begin(),lane_safe_x[1].end());
    plot_items.safeline[1].y.assign(lane_safe_y[1].begin(),lane_safe_y[1].end());
    // std::cout << " x y :" <<std::endl;
    // for(auto i=0; i<plot_items.safeline[0].x.size();++i)
    // {
    //     std::cout << plot_items.safeline[0].x[i] << " " << plot_items.safeline[0].y[i] <<std::endl;
    // }


}

void plot_box(std::vector<double> cent_x, std::vector<double> cent_y, std::vector<double>heading, std::vector<int> type, const enum ObsType &obs_type,PlotItems &plot_items)
{
    // cent_x= {0, 10, 23, 7 };
    // cent_y= {0, 1,  -1, 2 };
    // heading={0, 10, -10, 9};
    std::vector<std::vector<double>> box_conner_fl; 
    std::vector<std::vector<double>> box_conner_fr; 
    std::vector<std::vector<double>> box_conner_rl;
    std::vector<std::vector<double>> box_conner_rr; 

    

    // for(uint32 idx=0; idx < cent_x.size(); ++idx) // iterate all obstacle
    // {
    //     std::vector<std::vector<double>> corners;
    //     if(cent_x.size()==0) 
    //     {
    //         std::cout << "plot_box: no input" << std::endl;
    //         return;
    //     }
       
    //     if(type[idx] ==1 )
    //     {
    //         Box2d box({cent_x[idx], cent_y[idx]}, heading[idx], 4.68, 1.93); // vehicle
    //         box.GetAllCorners(&corners);
    //     }
            
    //     else
    //     {
    //         Box2d box({cent_x[idx], cent_y[idx]}, heading[idx], 1, 1); // pedestrian
    //         box.GetAllCorners(&corners);
    //     }
    //    // return;
        
    //    // corners.push_back(corners[0]); // pad first point to plot a whole box
        
    //     // for(uint32 c_idx=0; c_idx < corners.size();++c_idx)
    //     {
    //         // box_xs[idx].push_back(corners[c_idx][1] *-1);
    //         // box_ys[idx].push_back(corners[c_idx][0]);
    //         // box_xs[idx].push_back(corners[c_idx][0]);
    //         // box_ys[idx].push_back(corners[c_idx][1]);
    //         box_conner_fl.push_back({corners[0][0],corners[0][1]});
    //         box_conner_fr.push_back({corners[1][0],corners[1][1]});
    //         box_conner_rr.push_back({corners[2][0],corners[2][1]});
    //         box_conner_rl.push_back({corners[3][0],corners[3][1]});
    //     }
    //     plot_items.box_obstacle_corner[0].x.assign(box_conner_fl[0].)
        
    // }
    // std::cout << "cent_X: " <<cent_x.size()<<std::endl;
    // for(uint32 idx=0; idx < cent_x.size(); ++idx)
    // {
        // std::cout << "x0: " << box_xs[idx][0] <<"y0: "<<box_ys[idx][0]<<std::endl;
        // std::cout << "x1: " << box_xs[idx][1] <<"y1: "<<box_ys[idx][1]<<std::endl;
        // std::cout << "x2: " << box_xs[idx][2] <<"y2: "<<box_ys[idx][2]<<std::endl;
        // std::cout << "x3: " << box_xs[idx][3] <<"y3: "<<box_ys[idx][3]<<std::endl;

        // if(obs_type == ObsType::Ego_Vehicle)
        //     // plt::plot(box_xs[idx],box_ys[idx],"c-");

        // if(obs_type == ObsType::CIPV_V1)
        //     // plt::plot(box_xs[idx],box_ys[idx],"b-");

        // if(obs_type == ObsType::CIPV_V2)
        //     // plt::plot(box_xs[idx],box_ys[idx],"b--");

        // if(obs_type == ObsType::NotImpor_Obs)
        //     // plt::plot(box_xs[idx],box_ys[idx],"b--");

    // }

}


void plot_vehicleCoordi_wind(const hdMapTrajectory &hdMapTrajectory,const DecisionToPC &rev_DecisionToPC_data, const objSecList &selectObj, PlotItems &plot_items){
    
    std::vector<double> local_x; //ego car position with heading
    std::vector<double> local_y;

    std::vector<double> ref_path_x;
    std::vector<double> ref_path_y;
    std::vector<double> ref_path_h;

    std::vector<double> dis_cipv_left_x;
    std::vector<double> dis_cipv_left_y;
    std::vector<double> dis_cipv_left_h;
    std::vector<int> dis_cipv_left_type;

    std::vector<double> dis_cipv_mid_x;
    std::vector<double> dis_cipv_mid_y;
    std::vector<double> dis_cipv_mid_h;
    std::vector<int> dis_cipv_mid_type;

    std::vector<double> dis_cipv_right_x;
    std::vector<double> dis_cipv_right_y;
    std::vector<double> dis_cipv_right_h;
    std::vector<int> dis_cipv_right_type;

    std::vector<double> obj_list_cent_x;
    std::vector<double> obj_list_cent_y;
    std::vector<double> obj_list_heading;
    std::vector<int> obj_list_type;
    
    Dt_RECORD_EnvModelInfos envModelInfo = rev_DecisionToPC_data.my_envModelInfo;
    Dt_RECORD_HdmapInfo hdmapInfo = rev_DecisionToPC_data.my_hdmapInfo;
    // refpath
    for(unsigned int seg_idx=0; seg_idx< hdMapTrajectory.pathLane[0].segNum; seg_idx++)
    for(unsigned int node_idx=0; node_idx< hdMapTrajectory.pathLane[0].hdmapPathInfo[seg_idx].laneInfos[0].nodeNum; node_idx+=5)
    {
         ref_path_x.push_back(hdMapTrajectory.pathLane[0].hdmapPathInfo[seg_idx].laneInfos[0].laneNodeInfos[node_idx].y  * -1);
         ref_path_y.push_back(hdMapTrajectory.pathLane[0].hdmapPathInfo[seg_idx].laneInfos[0].laneNodeInfos[node_idx].x);
         ref_path_h.push_back(hdMapTrajectory.pathLane[0].hdmapPathInfo[seg_idx].laneInfos[0].laneNodeInfos[node_idx].heading);
    }

    if(selectObj.frontMid.postion ==1 )// CIPV_Mid
    {
        dis_cipv_mid_x.push_back(selectObj.frontMid.obj.pos_y * -1);
        dis_cipv_mid_y.push_back(selectObj.frontMid.obj.pos_x);
        dis_cipv_mid_h.push_back(selectObj.frontMid.obj.heading);
        dis_cipv_mid_type.push_back(selectObj.frontMid.obj.type);
    }

    if( selectObj.frontMidLeft.postion ==1)  // CIPV_left
    {
        dis_cipv_left_x.push_back(selectObj.frontMidLeft .obj.pos_y * -1);
        dis_cipv_left_y.push_back(selectObj.frontMidLeft.obj.pos_x);
        dis_cipv_left_h.push_back(selectObj.frontMidLeft.obj.heading);
        dis_cipv_left_type.push_back(selectObj.frontMidLeft.obj.type);
      

    }
     if(selectObj.frontMidRight.postion ==1)// CIPV_right
    {
        dis_cipv_right_x.push_back(selectObj.frontMidRight.obj.pos_y * -1);
        dis_cipv_right_y.push_back(selectObj.frontMidRight.obj.pos_x);
        dis_cipv_right_h.push_back(selectObj.frontMidRight.obj.heading);
        dis_cipv_right_type.push_back(selectObj.frontMidRight.obj.type);
      

    }

    if(selectObj.frontLeft.postion ==1  ) // CIPV_left
    {
        dis_cipv_mid_x.push_back(selectObj.frontLeft.obj.pos_y * -1);
        dis_cipv_mid_y.push_back(selectObj.frontLeft.obj.pos_x);
        dis_cipv_mid_h.push_back(selectObj.frontLeft.obj.heading);
        dis_cipv_mid_type.push_back((int)selectObj.frontLeft.obj.type);
    }
    if( selectObj.frontRight.postion == 1 ) // CIPV_right
    {
        dis_cipv_mid_x.push_back(selectObj.frontRight.obj.pos_y * -1);
        dis_cipv_mid_y.push_back(selectObj.frontRight.obj.pos_x);
        dis_cipv_mid_h.push_back(selectObj.frontRight.obj.heading);
        dis_cipv_mid_type.push_back((int)selectObj.frontRight.obj.type);
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

   
    // plot refline 
    plot_items.refline.x.assign(ref_path_x.begin(), ref_path_x.end());
    plot_items.refline.y.assign(ref_path_y.begin(), ref_path_y.end());


    // plot all obj_center // blue
    plot_items.center_obstacle.x.assign(obj_list_cent_x.begin(), obj_list_cent_x.end());
    plot_items.center_obstacle.y.assign(obj_list_cent_y.begin(), obj_list_cent_y.end());

    // plot cipvLeft  // red <
    plot_items.cipv[0].x.assign(dis_cipv_left_x.begin(), dis_cipv_left_x.end());
    plot_items.cipv[0].y.assign(dis_cipv_left_y.begin(), dis_cipv_left_y.end());
 
    // plot cipvMid  // red ^
    plot_items.cipv[1].x.assign(dis_cipv_mid_x.begin(), dis_cipv_mid_x.end());
    plot_items.cipv[1].y.assign(dis_cipv_mid_y.begin(), dis_cipv_mid_y.end());

    // plot cipvRight  // red >
    plot_items.cipv[2].x.assign(dis_cipv_right_x.begin(), dis_cipv_right_x.end());
    plot_items.cipv[2].y.assign(dis_cipv_right_y.begin(), dis_cipv_right_y.end());

    local_x.push_back(0);
    local_y.push_back(0);
    double head_err;
    if(hdmapInfo.planpath)
        head_err = hdMapTrajectory.pathLane[0].hdmapPathInfo[0].laneInfos[0].laneNodeInfos[0].heading - rev_DecisionToPC_data.my_localizationInfo.yaw;
    else
        head_err = hdMapTrajectory.localPath[0].hdmapPathInfo[0].laneInfos[0].laneNodeInfos[0].heading -rev_DecisionToPC_data.my_localizationInfo.yaw; 
    double head_x = 2 * cos(head_err/180.0 * M_PI);
    double head_y = 2 * sin(head_err/180.0 * M_PI);
    local_x.push_back(head_x);
    local_y.push_back(head_y);

    plot_items.ego_position.x.assign(local_x.begin(),local_x.end());
    plot_items.ego_position.y.assign(local_y.begin(),local_y.end()); 


    

    // plt::plot(ref_path_x, ref_path_y,"y--");
    // plt::plot(obj_list_cent_x, obj_list_cent_y,"bo");
    // plt::plot(dis_cipv_1_x, dis_cipv_1_y,"ro");
    // plt::plot(dis_cipv_2_x, dis_cipv_2_y,"ro");


    plot_lane(hdMapTrajectory, rev_DecisionToPC_data, plot_items);
    // plot_box(std::vector<double>{0},std::vector<double>{0},std::vector<double>{0}, std::vector<int>{1},ObsType::Ego_Vehicle, plot_items);

    // plot_box(obj_list_cent_x,obj_list_cent_y,obj_list_heading, obj_list_type, ObsType::NotImpor_Obs);
    // plot_box(dis_cipv_1_x,dis_cipv_1_y,dis_cipv_1_h, dis_cipv_1_type, ObsType::CIPV_V1);
    // plot_box(dis_cipv_2_x,dis_cipv_2_y,dis_cipv_2_h, dis_cipv_2_type, ObsType::CIPV_V2);

    plot_gridmap(envModelInfo, plot_items);
    // plot_box(obj_list_cent_x,obj_list_cent_y,obj_list_heading, ObsType::NotImpor_Obs);
}

void plot_infoList_wind(const DecisionToPC &rev_DecisionToPC_data){
   
    std::vector<string> DisMode{"Replay" , "Vehicle"};
    std::vector<string> DecisionCommand{"Invalid" , "Cruise", "Follow", "Left Change", " Right Change", "Left Avoid",\
         "Right Avoid", "Stop", "AEB", "Right Large Curv", "LeftCurv"};//3：左变道；4：右变道；5：左避障；6：右避障；7：停车；8：AEB；9：右转大曲率；10：左转大曲率
    plt::text(1,18, "Vehicle / Replay: " + DisMode[0]);
    plt::text(1,16, "On Path: " + std::to_string(rev_DecisionToPC_data.my_hdmapInfo.planpath));
    plt::text(1,14, "Vehicle Speed (KPH): " + std::to_string(rev_DecisionToPC_data.my_vehicleInfo.ESP_VehSpd * 3.6));
    plt::text(1,12, "Vehicle Posi: " + std::to_string(rev_DecisionToPC_data.my_localizationInfo.LocalizationResult.x) + " " +\
        std::to_string(rev_DecisionToPC_data.my_localizationInfo.LocalizationResult.y) + " " +  std::to_string(rev_DecisionToPC_data.my_localizationInfo.yaw));
    plt::text(1,10, "Decision Command: " + std::to_string(rev_DecisionToPC_data.my_trajectoryPointsInfo.decision)+ " " +DecisionCommand[rev_DecisionToPC_data.my_trajectoryPointsInfo.decision]);
    plt::text(1,8, "Sys Command: " + std::to_string(rev_DecisionToPC_data.my_sysmgtAvpCmdInfo.TBOX_AVPModKey));

}
void plot_mapCoordi_wind(const DecisionToPC &rev_DecisionToPC_data, PlotItems  &plot_items){
    Dt_RECORD_LocalizationInfo localInfos =rev_DecisionToPC_data.my_localizationInfo;
    Dt_RECORD_HdmapFrontPLane globePLane= rev_DecisionToPC_data.my_hdmapFrontPLaneInfo;
	Dt_RECORD_HdmapLocalLane my_hdmapLocalLaneInfo  = rev_DecisionToPC_data.my_hdmapLocalLaneInfo;

    static std::vector<std::vector<double>> path_seg_X(3);
    static std::vector<std::vector<double>> path_seg_Y(3);
    static std::vector<std::vector<double>> path_seg_H(3);
    std::vector<double> path_loc_seg_X;
    std::vector<double> path_loc_seg_Y;
    std::vector<double> path_loc_seg_H;
    std::vector<double>loc_X;
    std::vector<double>loc_Y;
    double loc_x =rev_DecisionToPC_data.my_localizationInfo.LocalizationResult.x;
    double loc_y =rev_DecisionToPC_data.my_localizationInfo.LocalizationResult.y;
    double cur_range = 20; 
    // std::cout <<"std::abs(loc_x - cur_range)" << std::abs(loc_x - cur_range) << " "<<std::abs(loc_y - cur_range) <<std::endl;
    // if(std::abs(loc_x - cur_range)< cur_range/5 || std::abs(loc_y - cur_range)< cur_range/5 || std::abs(loc_x - cur_range) > cur_range || std::abs(loc_y - cur_range)> cur_range) 
    // {
    //     plt::subplot2grid(2,3,0,1,1,1);
    //     plt::xlim(loc_x- cur_range, loc_x + cur_range);
    //     plt::ylim(loc_y- cur_range, loc_y + cur_range);
    // }

    loc_X.push_back(localInfos.LocalizationResult.x);
    loc_Y.push_back(localInfos.LocalizationResult.y);
    // std::cout <<"rev_DecisionToPC_data.my_hdmapInfo.planpath:" <<(double)rev_DecisionToPC_data.my_hdmapInfo.planpath <<std::endl;
    if(rev_DecisionToPC_data.my_hdmapInfo.planpath) //on path
    {
        for(unsigned int lane_seg_idx=0; lane_seg_idx < globePLane.plan_seg_count; ++lane_seg_idx)
        {
            std::vector<double> _x;
            std::vector<double> _y;
            std::vector<double> _heading;
            for(unsigned int node_idx=0; node_idx<globePLane.PlanSeg[lane_seg_idx].Lane[0].node_count; node_idx+=2) // scatter 2 times
            {
            
                _x.push_back(globePLane.PlanSeg[lane_seg_idx].Lane[0].LaneNode[node_idx].hdmap_x);
                _y.push_back(globePLane.PlanSeg[lane_seg_idx].Lane[0].LaneNode[node_idx].hdmap_y);
                _heading.push_back(globePLane.PlanSeg[lane_seg_idx].Lane[0].LaneNode[node_idx].heading);
            }
            path_seg_X[lane_seg_idx] = _x;
            path_seg_Y[lane_seg_idx] = _y;
            path_seg_H[lane_seg_idx] = _heading;
        }
    }
    else // not on path
    {
        std::vector<double> _x;
        std::vector<double> _y;
        std::vector<double> _heading;
        for(unsigned int node_idx = 0; node_idx< my_hdmapLocalLaneInfo.LocalLane[0].node_count; node_idx+=2)
        {
            _x.push_back(my_hdmapLocalLaneInfo.LocalLane[0].LaneNode[node_idx].hdmap_x);
            _y.push_back(my_hdmapLocalLaneInfo.LocalLane[0].LaneNode[node_idx].hdmap_y);
            _heading.push_back(my_hdmapLocalLaneInfo.LocalLane[0].LaneNode[node_idx].heading );
        }
        path_loc_seg_X = _x;
        path_loc_seg_Y = _y;
        path_loc_seg_H = _heading;

    }
    

    // plot refpath (segment 0->2)in map coordiante 
    plot_items.refline_mapCoor[0].x.assign(path_seg_X[0].begin(),path_seg_X[0].end());
    plot_items.refline_mapCoor[0].y.assign(path_seg_Y[0].begin(),path_seg_Y[0].end());
    plot_items.refline_mapCoor[1].x.assign(path_seg_X[1].begin(),path_seg_X[1].end());
    plot_items.refline_mapCoor[1].y.assign(path_seg_Y[1].begin(),path_seg_Y[1].end());
    plot_items.refline_mapCoor[2].x.assign(path_seg_X[2].begin(),path_seg_X[2].end());
    plot_items.refline_mapCoor[2].y.assign(path_seg_Y[2].begin(),path_seg_Y[2].end());

     // plot localpath in map coordiante 
    plot_items.refline_local_mapCoor.x.assign(path_loc_seg_X.begin(), path_loc_seg_X.end());
    plot_items.refline_local_mapCoor.y.assign(path_loc_seg_Y.begin(), path_loc_seg_Y.end());


    // plot refpath (segment 0->2)in map coordiante 
    plot_items.ego_position_mapCoor.x.assign(loc_X.begin(), loc_X.end());
    plot_items.ego_position_mapCoor.y.assign(loc_Y.begin(), loc_Y.end());
    

    // plt::plot(path_seg_X[lane_seg_idx], path_seg_Y[lane_seg_idx],"y--");
    // plt::plot(loc_X, loc_Y,"ro");

}


void plot_globalCoordi_wind(const Dt_RECORD_LocalizationInfo &localInfos, const Dt_RECORD_HdmapInfo &hdmapInfos, PlotItems  &plot_items){

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

    // if(loc_X[0]>0 && loc_Y[0]>0)
    // {
    //     plt::xlim(-10,110);
    //     plt::ylim(60,150); 
    //     plt::plot(global_path_x_gwh, global_path_y_gwh,"b");
       
    // }
    // else
    // {
    //     plt::xlim(-140,10);
    //     plt::ylim(-50,30);
    //     plt::plot(global_path_x_lg, global_path_y_lg,"b");
    // }
    start_X.push_back(hdmapInfos.origin_x);
    start_Y.push_back(hdmapInfos.origin_y);
    end_X.push_back(hdmapInfos.goal_x);
    end_Y.push_back(hdmapInfos.goal_y);
    

    // plot ego position in global map
    plot_items.ego_position_globalmap.x = loc_X;
    plot_items.ego_position_globalmap.y = loc_Y;
    
    // plot start point in global map
    plot_items.start_point_globalmap.x = start_X;
    plot_items.start_point_globalmap.y = start_Y;

    // plot end point in global map
    plot_items.end_point_globalmap.x = end_X;
    plot_items.end_point_globalmap.y = end_Y;



    // plt::plot(loc_X, loc_Y,"ro");
    // plt::plot(start_X, start_Y,"or");
    // plt::plot(end_X, end_Y,"or");
    // plt::text(start_X[0]+1,start_Y[0]+1,"Start");
    // plt::text(end_X[0]+1,end_Y[0]+1,"End");

}

int plot(const DecisionToPC &rev_DecisionToPC_data, const hdMapTrajectory &hdMapTrajectory, const objSecList &selectObj, PlotItems  &plot_items) {
    // plt::ion();
    // std::cout<<"data size: " <<rev_DecisionToPC_data.my_envModelInfo.frame_index <<std::endl;
    thread t1(plot_vehicleCoordi_wind, hdMapTrajectory, rev_DecisionToPC_data, selectObj, ref(plot_items));
    // thread t2(plot_globalCoordi_wind, rev_DecisionToPC_data.my_localizationInfo, rev_DecisionToPC_data.my_hdmapInfo, ref(plot_items) );
    // thread t3(plot_mapCoordi_wind, rev_DecisionToPC_data, ref(plot_items) );
    t1.join();
    // t2.join();
    // t3.join();
    // plot_vehicleCoordi_wind(hdMapTrajectory, rev_DecisionToPC_data, selectObj, plot_items);
    plot_globalCoordi_wind(rev_DecisionToPC_data.my_localizationInfo, rev_DecisionToPC_data.my_hdmapInfo, plot_items);
    plot_mapCoordi_wind(rev_DecisionToPC_data,  plot_items);
    // plot_infoList_wind(rev_DecisionToPC_data);
    plt::pause(0.001);
    // plt::clf();

}
