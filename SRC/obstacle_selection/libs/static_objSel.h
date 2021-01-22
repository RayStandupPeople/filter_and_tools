#pragma once
#include <iostream>
#include <vector>
#include <math.h>
#include "string.h"
#include "../../common/libs/Rte_Type.h"
#include "../../common/libs/user_struct.h"
#include "../../common/libs/decision_struct.h"

class StaticDecision{
public:

    void segmentCollisionCheck(float veh_heading,float veh_width, float veh_rear_axel_to_head, float veh_rear_axel_to_tail, float collision_dist, laneInfo single_laneinfo, Dt_RECORD_EnvModelInfos *envModelInfo, int *coll_flag, float64 *position);
    void segmentCollisionCheckWithDetail(float veh_heading,float veh_width, float veh_rear_axel_to_head, \
      float veh_rear_axel_to_tail, float collision_dist, laneInfo single_laneinfo, Dt_RECORD_EnvModelInfos *envModelInfo,  \
      std::vector<int> &coll_flag, float64 *position, const double &lane_width,  decision_info *decisionInfo);
   
    void RouteObjDectbyGrid(float veh_heading,int onpath, hdMapTrajectory *Trajectory, Dt_RECORD_HdmapInfo *hdmapInfos, Dt_RECORD_EnvModelInfos *envModelInfo, gridmap_coll_obj *coll_obj,\
		Dt_RECORD_HdmapFrontPLane *globePLane, 	Dt_RECORD_HdmapLocalLane *localPLanne,  decision_info *decisionInfo);
   
    static int collisionCheckInGridMap(const double ego_x, const double ego_y, const double ego_yaw, const double veh_width,
	    const double veh_rear_axel_to_head, const double veh_rear_axel_to_tail, const double collision_dist,
	    const Dt_ARRAY_80000_ObstacleGridMap ObstacleGridMap);
    
    static std::vector<int>  collisionCheckInGridMapWithDetail(const double ego_x, const double ego_y, const double ego_yaw, const double lanewidth, const double veh_width,
	    const double veh_rear_axel_to_head, const double veh_rear_axel_to_tail, const double collision_dist,
	    const Dt_ARRAY_80000_ObstacleGridMap ObstacleGridMap);
};