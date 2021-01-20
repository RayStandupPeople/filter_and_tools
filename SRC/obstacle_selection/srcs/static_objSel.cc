#include "../libs/static_objSel.h"

/***************************************************************************************************
* 功  能: 基于栅格地图信息对车辆行驶前方地图模块发布的单端道路各路点是否存在碰撞进行检测
* 输  入: Dt_RECORD_EnvModelInfos *envModelInfo   环境建模信息
		  float veh_width   车辆宽度
		  float veh_rear_axel_to_head 后轴中心到车头长度
		  float veh_rear_axel_to_tail 后轴中心到车位长度
		  float collision_dist 车辆膨胀宽度
		  laneInfo single_laneinfo 单端道路路点信息
* 输  出: int                  发送给泊车模块的可泊车位信息
输入&输出       parkStartFlag                  泊车开始标志位
调用函数：无
更新:    V2.0 add SideAvoid Logic
***************************************************************************************************/
void StaticDecision::segmentCollisionCheckWithDetail(float veh_heading, float veh_width, float veh_rear_axel_to_head, \
	 float veh_rear_axel_to_tail, float collision_dist, laneInfo single_laneinfo, Dt_RECORD_EnvModelInfos *envModelInfo,\
	 std::vector<int> &coll_flag, float64 *position, const double &lane_width,  decision_info *decisionInfo)
{
	std::vector<std::vector<double>> detect_range = {{0, 199}, {0, 355}}; // default range {lateral, longitudinal}
	if (decisionInfo->decision_command== LEFTAVOID)
	{
		detect_range[0] = {100 + 12, 199};  // just detect right side 
		detect_range[1] = {0, 399};    //  whole longitudinal range
	}

	if (decisionInfo->decision_command == RIGHTAVOID) // Avoiding
	{
		detect_range[0] = {0, 99 - 12};     // just detect left side
		detect_range[1] = {0, 399};    //  whole longitudinal range
	}


	for (uint32 nodenum = 0; nodenum < single_laneinfo.nodeNum; nodenum++) //隔1m进行检测
	{
		float LocDeltaHeading = 0;
		if (nodenum == 0)
		{
			LocDeltaHeading = 90;
		}
		else
		{
			LocDeltaHeading =  single_laneinfo.laneNodeInfos[nodenum].heading - veh_heading + 90; // zlm temp del
			// LocDeltaHeading =  single_laneinfo.laneNodeInfos[nodenum].heading + 90;

		}
		//测试，lkw,20201225
		// LocDeltaHeading = 90;//???
		if (LocDeltaHeading > 360)
		{
			LocDeltaHeading = (LocDeltaHeading - 360) / 180 * M_PI;
		}
		else if (LocDeltaHeading < 0)
		{
			LocDeltaHeading = (LocDeltaHeading + 360) / 180 * M_PI;
		}
		else
		{
			LocDeltaHeading = LocDeltaHeading / 180 * M_PI;
		}
		// DEBUG("LocDeltaHeading = %f\r\n", LocDeltaHeading * 180/ M_PI);
		//计算轨迹点在栅格图下的坐标
		float Loc_ego_x = 10 - single_laneinfo.laneNodeInfos[nodenum].y;
		float Loc_ego_y = 40 - single_laneinfo.laneNodeInfos[nodenum].x;
		#if 1 // zlm 2021- 0107 add new fun(support in-lane obstacle avoid)
		coll_flag = StaticDecision::collisionCheckInGridMapWithDetail(Loc_ego_x, Loc_ego_y, LocDeltaHeading, lane_width, veh_width,
														 veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, envModelInfo->ObstacleGridMap, detect_range);
   		#endif

    	#if 0
		*coll_flag = StaticDecision::collisionCheckInGridMap(Loc_ego_x, Loc_ego_y, LocDeltaHeading, veh_width,
														 veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, envModelInfo->ObstacleGridMap);
   		#endif

		#if 0
		*coll_flag = CommonMath::collisionCheckInGridMapWithBFS(Loc_ego_x, Loc_ego_y, LocDeltaHeading, veh_width,
																veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, envModelInfo->ObstacleGridMap);
    	#endif

		// if (*coll_flag != 0)    // zlm 2021-0107 comment
		for(uint32 part_idx =0; part_idx<3;++part_idx)
		{
			if(coll_flag[part_idx]!= 0 )
			{
				DEBUG("single_laneinfo.laneNodeInfos[%d].x = %f, single_laneinfo.laneNodeInfos[%d].y = %f, single_laneinfo.laneNodeInfos[%d].s = %f \r\n", nodenum, single_laneinfo.laneNodeInfos[nodenum].x, nodenum, single_laneinfo.laneNodeInfos[nodenum].y, nodenum, single_laneinfo.laneNodeInfos[nodenum].s);
				position[0] = single_laneinfo.laneNodeInfos[nodenum].x;
				position[1] = single_laneinfo.laneNodeInfos[nodenum].y;
				position[2] = single_laneinfo.laneNodeInfos[nodenum].x + veh_rear_axel_to_head;//lkw,20201229,距离后轴中心距离
				return; // no matter which lane part make collision, break iterate
			}
			else
			{
				position[0] = 0;
				position[1] = 0;
				position[2] = 0;	
			}
		}
	}
}


/***************************************************************************************************
* 功  能: 基于栅格地图信息对车辆行驶前方地图模块发布的单端道路各路点是否存在碰撞进行检测
* 输  入: Dt_RECORD_EnvModelInfos *envModelInfo   环境建模信息
		  float veh_width   车辆宽度
		  float veh_rear_axel_to_head 后轴中心到车头长度
		  float veh_rear_axel_to_tail 后轴中心到车位长度
		  float collision_dist 车辆膨胀宽度
		  laneInfo single_laneinfo 单端道路路点信息
* 输  出: int                  发送给泊车模块的可泊车位信息
输入&输出       parkStartFlag                  泊车开始标志位
调用函数：无
***************************************************************************************************/
void StaticDecision::segmentCollisionCheck(float veh_heading, float veh_width, float veh_rear_axel_to_head,\
	 float veh_rear_axel_to_tail, float collision_dist, laneInfo single_laneinfo, Dt_RECORD_EnvModelInfos *envModelInfo,\
	 int *coll_flag, float64 *position)
{
	for (uint32 nodenum = 0; nodenum < single_laneinfo.nodeNum; nodenum++) //隔1m进行检测
	{
		float LocDeltaHeading = 0;
		if (nodenum == 0)
		{
			LocDeltaHeading = 90;
		}
		else
		{
			LocDeltaHeading =  single_laneinfo.laneNodeInfos[nodenum].heading - veh_heading + 90;
		}
		//测试，lkw,20201225
		LocDeltaHeading = 90;
		if (LocDeltaHeading > 360)
		{
			LocDeltaHeading = (LocDeltaHeading - 360) / 180 * M_PI;
		}
		else if (LocDeltaHeading < 0)
		{
			LocDeltaHeading = (LocDeltaHeading + 360) / 180 * M_PI;
		}
		else
		{
			LocDeltaHeading = LocDeltaHeading / 180 * M_PI;
		}
		//计算轨迹点在栅格图下的坐标
		float Loc_ego_x = 10 - single_laneinfo.laneNodeInfos[nodenum].y;
		float Loc_ego_y = 40 - single_laneinfo.laneNodeInfos[nodenum].x;

    	#if 1
		*coll_flag = StaticDecision::collisionCheckInGridMap(Loc_ego_x, Loc_ego_y, LocDeltaHeading, veh_width,
														 veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, envModelInfo->ObstacleGridMap);
   		#endif

		#if 0
		*coll_flag = CommonMath::collisionCheckInGridMapWithBFS(Loc_ego_x, Loc_ego_y, LocDeltaHeading, veh_width,
																veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, envModelInfo->ObstacleGridMap);
    	#endif

		if (*coll_flag != 0)    
		{
			DEBUG("single_laneinfo.laneNodeInfos[%d].x = %f, single_laneinfo.laneNodeInfos[%d].y = %f, single_laneinfo.laneNodeInfos[%d].s = %f \r\n", nodenum, single_laneinfo.laneNodeInfos[nodenum].x, nodenum, single_laneinfo.laneNodeInfos[nodenum].y, nodenum, single_laneinfo.laneNodeInfos[nodenum].s);
			position[0] = single_laneinfo.laneNodeInfos[nodenum].x;
			position[1] = single_laneinfo.laneNodeInfos[nodenum].y;
			position[2] = single_laneinfo.laneNodeInfos[nodenum].x + veh_rear_axel_to_head;//lkw,20201229,距离后轴中心距离
			break; 
		}
		else
		{
			position[0] = 0;
			position[1] = 0;
			position[2] = 0;	
		}
	}
}

//行驶轨迹上障碍物判断
#define zhulimin
#ifdef zhulimin
void StaticDecision::RouteObjDectbyGrid(float veh_heading, int onpath, hdMapTrajectory *Trajectory, Dt_RECORD_HdmapInfo *hdmapInfos, Dt_RECORD_EnvModelInfos *envModelInfo, gridmap_coll_obj *coll_obj,\
		Dt_RECORD_HdmapFrontPLane *globePLane, 	Dt_RECORD_HdmapLocalLane *localPLanne,  decision_info *decisionInfo)
{
	float veh_width = 1.95;
	float veh_rear_axel_to_head = 3.75;
	float veh_rear_axel_to_tail = 0.95;
	float collision_dist = 0.2;

	//障碍物标志位
	std::vector<int> obj_flag_mid_array;   //zlm 2021-0107 add
	int obj_flag_mid = 0, obj_flag_left = 0, obj_flag_right = 0;
	float64 temp_position[3] = {0};//0---x,1---y,2---dis;
	

	// 轨迹中存在LC_flag的标志位
	int lc_exist_flag = 0;
	uint32 lc_exist_segnum = 0;
	int lc_exist_type = 0; //0->left  1->right
	static double valid_lane_width;
	double cur_lane_width;
	if(onpath)
		cur_lane_width = globePLane->PlanSeg[0].Lane[0].lane_width;
	else
		cur_lane_width = localPLanne->LocalLane[0].lane_width;

	if(cur_lane_width > 0) // save valid lane
		valid_lane_width = cur_lane_width;
	else 				   // read valid lane
		cur_lane_width = valid_lane_width;
		
	double detect_range = 355;
	if (onpath == TRUE || decisionInfo->decision_command == LEFTAVOID || decisionInfo->decision_command == RIGHTAVOID)
	{
		//在全局轨迹上
		for (uint32 segNum = 0; segNum < Trajectory->pathLane[0].segNum; segNum++)
		{
			DEBUG("Trajectory->pathLane[0].hdmapPathInfo[0].laneInfos[0].nodeNum= %d\r\n", Trajectory->pathLane[0].hdmapPathInfo[0].laneInfos[0].nodeNum);
			if (Trajectory->pathLane[0].hdmapPathInfo[0].laneInfos[0].nodeNum <= 0)
			{
				//DEBUG("decision=hdmaperror \r\n");
				return;
			}

			segmentCollisionCheckWithDetail(veh_heading, veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, \
				collision_dist, Trajectory->pathLane[0].hdmapPathInfo[segNum].laneInfos[0], envModelInfo, \
				obj_flag_mid_array, temp_position, cur_lane_width, decisionInfo);

			// if (obj_flag_mid != 0)  zlm 2021-0107 comment
			for(uint32 part_idx=0; part_idx<3; ++part_idx) //iterate each part of one lane
			{
				if(obj_flag_mid_array[part_idx]!=0)
				{
					if(part_idx==1)
					{
						DEBUG("===decision.cpp===there have a mid obj...\r\n");
						coll_obj->obj_mid_flag = obj_flag_mid_array[part_idx];
						coll_obj->x_m = temp_position[0];
						coll_obj->y_m = temp_position[1];
						coll_obj->s_m = temp_position[2];
						DEBUG("[===decision.cpp===]:coll_obj->x_m = %f\r\n", coll_obj->x_m);
						DEBUG("[===decision.cpp===]:coll_obj->y_m = %f\r\n", coll_obj->y_m);
						DEBUG("[===decision.cpp===]:coll_obj->s_m = %f\r\n", coll_obj->s_m);
					
					}
					if(part_idx==0)
					{
						DEBUG("===decision.cpp===there have a midLeft obj...\r\n");
						coll_obj->obj_midLeft_flag = obj_flag_mid_array[part_idx];
						coll_obj->x_m = temp_position[0];
						coll_obj->y_m = temp_position[1];
						coll_obj->s_m = temp_position[2];
						DEBUG("[===decision.cpp===]:coll_obj->x_m = %f\r\n", coll_obj->x_m);
						DEBUG("[===decision.cpp===]:coll_obj->y_m = %f\r\n", coll_obj->y_m);
						DEBUG("[===decision.cpp===]:coll_obj->s_m = %f\r\n", coll_obj->s_m);
					
					}
					if(part_idx==2)
					{
						DEBUG("===decision.cpp===there have a midRight obj...\r\n");
						coll_obj->obj_midRight_flag = obj_flag_mid_array[part_idx];
						coll_obj->x_m = temp_position[0];
						coll_obj->y_m = temp_position[1];
						coll_obj->s_m = temp_position[2];
						DEBUG("[===decision.cpp===]:coll_obj->x_m = %f\r\n", coll_obj->x_m);
						DEBUG("[===decision.cpp===]:coll_obj->y_m = %f\r\n", coll_obj->y_m);
						DEBUG("[===decision.cpp===]:coll_obj->s_m = %f\r\n", coll_obj->s_m);
						
					}
			    }
			}
			if(coll_obj->obj_mid_flag!=0) break; //current sgement make collision, quit iteration
		}

		//如果当前车道会发生碰撞，则判断左右车道障碍物
		// if (obj_flag_mid != 0)   // zlm comment, check side lane all the time
		{
		for (uint32 segNum = 0; segNum < Trajectory->pathLane[0].segNum; segNum++)
		{
			//判断当前段车道是否有左右车道
			DEBUG("Trajectory->pathLane[0].hdmapPathInfo[segNum].laneNum = %d \r\n", Trajectory->pathLane[0].hdmapPathInfo[segNum].laneNum);
			switch (Trajectory->pathLane[0].hdmapPathInfo[segNum].laneNum)
			{
			case 2: //只有左车道
				if (obj_flag_left == 0)
				{
					segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, Trajectory->pathLane[0].hdmapPathInfo[segNum].laneInfos[1], envModelInfo, &obj_flag_left, temp_position);
					if (obj_flag_left != 0)
					{
						DEBUG("===decision.cpp===there have a left obj...\r\n");
						coll_obj->obj_lef_flag = obj_flag_left;
						coll_obj->x_l = temp_position[0];
						coll_obj->y_l = temp_position[1];
						coll_obj->s_l = temp_position[2];
						DEBUG("[===decision.cpp===]:coll_obj->x_l = %f\r\n", coll_obj->x_l);
						DEBUG("[===decision.cpp===]:coll_obj->y_l = %f\r\n", coll_obj->y_l);
						DEBUG("[===decision.cpp===]:coll_obj->s_l = %f\r\n", coll_obj->s_l);
					}
				}
				break;
			case 3: //只有右车道
				if (obj_flag_right == 0)
				{
					segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, Trajectory->pathLane[0].hdmapPathInfo[segNum].laneInfos[2], envModelInfo, &obj_flag_right, temp_position);
					if (obj_flag_right != 0)
					{
						DEBUG("===decision.cpp===there have a right obj...\r\n");
						coll_obj->obj_rig_flag = obj_flag_right;
						coll_obj->x_r = temp_position[0];
						coll_obj->y_r = temp_position[1];
						coll_obj->s_r = temp_position[2];
						DEBUG("[===decision.cpp===]:coll_obj->x_r = %f\r\n", coll_obj->x_r);
						DEBUG("[===decision.cpp===]:coll_obj->y_r = %f\r\n", coll_obj->y_r);
						DEBUG("[===decision.cpp===]:coll_obj->s_r = %f\r\n", coll_obj->s_r);
					}
				}
				break;for (uint32 segNum = 0; segNum < Trajectory->localPath[0].segNum; segNum++)
		{
			//判断当前段车道是否有左右车道
			DEBUG("Trajectory->localPath[0].hdmapPathInfo[segNum].laneNum = %d \r\n", Trajectory->localPath[0].hdmapPathInfo[segNum].laneNum);
			switch (Trajectory->localPath[0].hdmapPathInfo[segNum].laneNum)
			{
			case 2: //只有左车道
				if (obj_flag_left == 0)
				{
					segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, Trajectory->localPath[0].hdmapPathInfo[segNum].laneInfos[1], envModelInfo, &obj_flag_left, temp_position);
					if (obj_flag_left != 0)
					{
						DEBUG("===decision.cpp===there have a left obj...\r\n");
						coll_obj->obj_lef_flag = obj_flag_left;
						coll_obj->x_l = temp_position[0];
						coll_obj->y_l = temp_position[1];
						coll_obj->s_l = temp_position[2];
						DEBUG("[===decision.cpp===]:coll_obj->x_l = %f\r\n", coll_obj->x_l);
						DEBUG("[===decision.cpp===]:coll_obj->y_l = %f\r\n", coll_obj->y_l);
						DEBUG("[===decision.cpp===]:coll_obj->s_l = %f\r\n", coll_obj->s_l);
					}
				}
				break;
			case 3: //只有右车道
				if (obj_flag_right == 0)
				{
					segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, Trajectory->localPath[0].hdmapPathInfo[segNum].laneInfos[2], envModelInfo, &obj_flag_right, temp_position);
					if (obj_flag_right != 0)
					{
						DEBUG("===decision.cpp===there have a right obj...\r\n");
						coll_obj->obj_rig_flag = obj_flag_right;
						coll_obj->x_r = temp_position[0];
						coll_obj->y_r = temp_position[1];
						coll_obj->s_r = temp_position[2];
						DEBUG("[===decision.cpp===]:coll_obj->x_r = %f\r\n", coll_obj->x_r);
						DEBUG("[===decision.cpp===]:coll_obj->y_r = %f\r\n", coll_obj->y_r);
						DEBUG("[===decision.cpp===]:coll_obj->s_r = %f\r\n", coll_obj->s_r);
					}
				}
				break;
			case 4: //左右车道均有
				if (obj_flag_left == 0)
				{
					segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, Trajectory->localPath[0].hdmapPathInfo[segNum].laneInfos[1], envModelInfo, &obj_flag_left, temp_position);
					if (obj_flag_left != 0)
					{
						DEBUG("===decision.cpp===there have a left obj...\r\n");
						coll_obj->obj_lef_flag = obj_flag_left;
						coll_obj->x_l = temp_position[0];
						coll_obj->y_l = temp_position[1];
						coll_obj->s_l = temp_position[2];
						DEBUG("[===decision.cpp===]:coll_obj->x_l = %f\r\n", coll_obj->x_l);
						DEBUG("[===decision.cpp===]:coll_obj->y_l = %f\r\n", coll_obj->y_l);
						DEBUG("[===decision.cpp===]:coll_obj->s_l = %f\r\n", coll_obj->s_l);
					}
				}
				if (obj_flag_right == 0)
				{
					segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, Trajectory->localPath[0].hdmapPathInfo[segNum].laneInfos[2], envModelInfo, &obj_flag_right, temp_position);
					if (obj_flag_right != 0)
					{
						DEBUG("===decision.cpp===there have a right obj...\r\n");
						coll_obj->obj_rig_flag = obj_flag_right;
						coll_obj->x_r = temp_position[0];
						coll_obj->y_r = temp_position[1];
						coll_obj->s_r = temp_position[2];
						DEBUG("[===decision.cpp===]:coll_obj->x_r = %f\r\n", coll_obj->x_r);
						DEBUG("[===decision.cpp===]:coll_obj->y_r = %f\r\n", coll_obj->y_r);
						DEBUG("[===decision.cpp===]:coll_obj->s_r = %f\r\n", coll_obj->s_r);
					}
				}
				break;
			default:
				break;
			}
			if (coll_obj->obj_lef_flag > 0 && coll_obj->obj_rig_flag > 0)
			{
				break;
			}
		}
				{
					segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, Trajectory->pathLane[0].hdmapPathInfo[segNum].laneInfos[1], envModelInfo, &obj_flag_left, temp_position);
					if (obj_flag_left != 0)
					{
						DEBUG("===decision.cpp===there have a left obj...\r\n");
						coll_obj->obj_lef_flag = obj_flag_left;
						coll_obj->x_l = temp_position[0];
						coll_obj->y_l = temp_position[1];
						coll_obj->s_l = temp_position[2];
						DEBUG("[===decision.cpp===]:coll_obj->x_l = %f\r\n", coll_obj->x_l);
						DEBUG("[===decision.cpp===]:coll_obj->y_l = %f\r\n", coll_obj->y_l);
						DEBUG("[===decision.cpp===]:coll_obj->s_l = %f\r\n", coll_obj->s_l);
					}
				}
				if (obj_flag_right == 0)
				{
					segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, Trajectory->pathLane[0].hdmapPathInfo[segNum].laneInfos[2], envModelInfo, &obj_flag_right, temp_position);
					if (obj_flag_right != 0)
					{
						DEBUG("===decision.cpp===there have a right obj...\r\n");
						coll_obj->obj_rig_flag = obj_flag_right;
						coll_obj->x_r = temp_position[0];
						coll_obj->y_r = temp_position[1];
						coll_obj->s_r = temp_position[2];
						DEBUG("[===decision.cpp===]:coll_obj->x_r = %f\r\n", coll_obj->x_r);
						DEBUG("[===decision.cpp===]:coll_obj->y_r = %f\r\n", coll_obj->y_r);
						DEBUG("[===decision.cpp===]:coll_obj->s_r = %f\r\n", coll_obj->s_r);
					}
				}
				break;
			default:
				break;
			}
			if (coll_obj->obj_lef_flag > 0 && coll_obj->obj_rig_flag > 0)
			{
				break;
			}
		}
		}

		//////////////////////////////////////
		// 被动变道轨迹进行处理   zlm  全局路径
		//////////////////////////////////////
		
		// 被动变道场景检测 
		// DEBUG("[passive lane change scenario Begin],basicInfo-> globePLane->plan_seg_count = %d \r\n", globePLane->plan_seg_count);
				
		for (uint32 segNum = 0; segNum < globePLane->plan_seg_count; segNum++)
		{
			if(globePLane->PlanSeg[segNum].Lane[0].change_lane_flag ==1) // 存在被动变道的路段
			{
				
				if (fabs(float(globePLane->PlanSeg[segNum].Lane[0].lane_NO)) < \
					fabs(float(globePLane->PlanSeg[segNum+1].Lane[0].lane_NO)))
				lc_exist_type =1; // 向右侧变道
				lc_exist_flag =1;
				lc_exist_segnum = segNum;
				DEBUG("[passive lane change scenario],basicInfo-> lc_exist_flag =%d , lc_exist_segnum= %d，lc_exist_type= %d\r\n", 
					lc_exist_flag, lc_exist_segnum, lc_exist_type);
			}
		}
		// 被动变道场景 并且前方发生碰撞
		if(lc_exist_flag == 1 && obj_flag_mid!=0)
		{
			DEBUG("[passive lane change scenario] now lc_exist_flag == 1 && obj_flag_mid!=0 \r\n");
			//变道前的碰撞检测与处理

			obj_flag_mid = 0; //状态清理
			obj_flag_left = 0; 
			obj_flag_right = 0; 
			

			for (uint32 segNum = 0; segNum <= lc_exist_segnum; segNum++)
			{
				//对本车道进行二次碰撞检测 
				if(obj_flag_mid == 0) 
					segmentCollisionCheckWithDetail(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, \
						collision_dist, Trajectory->pathLane[0].hdmapPathInfo[segNum].laneInfos[0], envModelInfo,\
					 	obj_flag_mid_array, temp_position, cur_lane_width, decisionInfo);
			}
			if(obj_flag_mid==0)   // 本车道处理
			{
				coll_obj->obj_mid_flag = 0;
				coll_obj->x_m = 0;
				coll_obj->y_m = 0;
				coll_obj->s_m = 0;
				DEBUG("[passive lane change scenario] second mid lane check, no collison...\r\n");
				// break;
			}
			for (uint32 segNum = 0; segNum <= lc_exist_segnum; segNum++) //相邻车道处理
			{
				//对相邻车道进行碰撞检测
				DEBUG("[Passive Lane change scenario, deal lc_exist_seg nearby lanes] : Trajectory->pathLane[0].hdmapPathInfo[segNum].laneNum = %d \r\n", \
					Trajectory->pathLane[0].hdmapPathInfo[segNum].laneNum);
				switch (Trajectory->pathLane[0].hdmapPathInfo[segNum].laneNum)
				{
					case 2: //只有左车道
						if (obj_flag_left == 0)
						{
							segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, Trajectory->pathLane[0].hdmapPathInfo[segNum].laneInfos[1], envModelInfo, &obj_flag_left, temp_position);
							if (obj_flag_left != 0)
							{
								DEBUG("[Passive Lane change scenario case2]:there have a left obj befor LC...\r\n");
								
								coll_obj->obj_lef_flag = obj_flag_left;
								coll_obj->x_l = temp_position[0];
								coll_obj->y_l = temp_position[1];
								coll_obj->s_l = temp_position[2];
								DEBUG("[Passive Lane change scenario lc_exist case2]:coll_obj->x_l = %f\r\n",coll_obj->x_l);
								DEBUG("[Passive Lane change scenario lc_exist case2]:coll_obj->y_l = %f\r\n", coll_obj->y_l);
								DEBUG("[Passive Lane change scenario lc_exist case2]:coll_obj->s_l = %f\r\n",coll_obj->s_l);
							}
						}
						break;
					case 3: //只有右车道
						if (obj_flag_right == 0)
						{
							segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, Trajectory->pathLane[0].hdmapPathInfo[segNum].laneInfos[2], envModelInfo, &obj_flag_right, temp_position);
							if (obj_flag_right != 0)
							{
								DEBUG("[Passive Lane change scenario case3]:there have a right obj befor LC...\r\n");
								coll_obj->obj_rig_flag = obj_flag_right;
								coll_obj->x_r = temp_position[0];
								coll_obj->y_r = temp_position[1];
								coll_obj->s_r = temp_position[2];
								DEBUG("[Passive Lane change scenario lc_exist case3]:coll_obj->x_r = %f\r\n",coll_obj->x_r);
								DEBUG("[Passive Lane change scenario lc_exist case3]:coll_obj->y_r = %f\r\n", coll_obj->y_r);
								DEBUG("[Passive Lane change scenario lc_exist case3]:coll_obj->s_r = %f\r\n",coll_obj->s_r);
							}
						}
						break;
					case 4: //左右车道均有
						if (obj_flag_left == 0)
						{
							segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, Trajectory->pathLane[0].hdmapPathInfo[segNum].laneInfos[1], envModelInfo, &obj_flag_left, temp_position);
							if (obj_flag_left != 0)
							{
								DEBUG("[Passive Lane change scenario case 4]:there have a left obj befor LC...\r\n");
								coll_obj->obj_lef_flag = obj_flag_left;
								coll_obj->x_l = temp_position[0];
								coll_obj->y_l = temp_position[1];
								coll_obj->s_l = temp_position[2];
								DEBUG("[Passive Lane change scenario lc_exist case4]:coll_obj->x_l = %f\r\n", coll_obj->x_l);
								DEBUG("[Passive Lane change scenario lc_exist case4]:coll_obj->y_l = %f\r\n", coll_obj->y_l);
								DEBUG("[Passive Lane change scenario lc_exist case4]:coll_obj->s_l = %f\r\n", coll_obj->s_l);
							}
						}
						if (obj_flag_right == 0)
						{
							segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, Trajectory->pathLane[0].hdmapPathInfo[segNum].laneInfos[2], envModelInfo, &obj_flag_right, temp_position);
							if (obj_flag_right != 0)
							{
								DEBUG("[Passive Lane change scenario case4]:there have a right obj befor LC...\r\n");
								coll_obj->obj_rig_flag = obj_flag_right;
								coll_obj->x_r = temp_position[0];
								coll_obj->y_r = temp_position[1];
								coll_obj->s_r = temp_position[2];
								DEBUG("[Passive Lane change scenario lc_exist case4]:coll_obj->x_r = %f\r\n", coll_obj->x_r);
								DEBUG("[Passive Lane change scenario lc_exist case4]:coll_obj->y_r = %f\r\n", coll_obj->y_r);
								DEBUG("[Passive Lane change scenario lc_exist case4]:coll_obj->s_r = %f\r\n", coll_obj->s_r);
							}
						}
						break;
					default:
						break;
				}
			}

			//变道后碰撞检测与处理
			DEBUG("[Passive Lane change scenario]:Trajectory->pathLane[0].segNum = %d \r\n", Trajectory->pathLane[0].segNum);
			for (uint32 segNum = lc_exist_segnum + 2; segNum < Trajectory->pathLane[0].segNum; segNum++)
			{
				if(lc_exist_type==0)//当前左变道
				{
					if(obj_flag_mid ==0)
					{
								DEBUG("[Passive Lane change scenario, lf change]:front no collision, check this seg right\r\n");
						if(Trajectory->pathLane[0].hdmapPathInfo[segNum].laneNum == 3 || Trajectory->pathLane[0].hdmapPathInfo[segNum].laneNum == 4)
							// 车辆变道前无障碍物，且本路段存在右侧车道，对右侧车道进行碰撞检测（即当前车辆的本车道远方）
						{
							segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, \
								collision_dist, Trajectory->pathLane[0].hdmapPathInfo[segNum].laneInfos[2], envModelInfo,\
								&obj_flag_mid, temp_position);
							if(obj_flag_mid != 0)
							{
								DEBUG("[Passive Lane change scenario, lf change]:there have a remote mid obj After LC...\r\n");
								coll_obj->obj_mid_flag = obj_flag_mid;
								coll_obj->x_m = temp_position[0];
								coll_obj->y_m = temp_position[1];
								coll_obj->s_m = temp_position[2];
								DEBUG("[Passive Lane change scenario lc_exist lf change]:coll_obj->x_m = %f\r\n", coll_obj->x_m);
								DEBUG("[Passive Lane change scenario lc_exist lf change]:coll_obj->y_m = %f\r\n", coll_obj->y_m);
								DEBUG("[Passive Lane change scenario lc_exist lf change]:coll_obj->s_m = %f\r\n", coll_obj->s_m);
							}
						}
					}
					// 检测本路段碰撞（即当前车辆的左侧车道远方）
					if(obj_flag_left==0)
					{
							DEBUG("[Passive Lane change scenario, lf change]:current left no collsion, check this seg ego\r\n");
							segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, \
							collision_dist, Trajectory->pathLane[0].hdmapPathInfo[segNum].laneInfos[0], envModelInfo,\
							&obj_flag_left, temp_position);
						if(obj_flag_left != 0)
						{
							DEBUG("[Passive Lane change scenario lf change]:there have a remote left obj After LC...\r\n");
							coll_obj->obj_lef_flag = obj_flag_left;
							coll_obj->x_l = temp_position[0];
							coll_obj->y_l = temp_position[1];
							coll_obj->s_l = temp_position[2];
							DEBUG("[Passive Lane change scenario lc_exist lf change]:coll_obj->x_l = %f\r\n", coll_obj->x_l);
							DEBUG("[Passive Lane change scenario lc_exist lf change]:coll_obj->y_l = %f\r\n", coll_obj->y_l);
							DEBUG("[Passive Lane change scenario lc_exist lf change]:coll_obj->s_l = %f\r\n", coll_obj->s_l);
						}
					}
					
				}
				if(lc_exist_type==1) //当前为右变道
				{
					if(obj_flag_mid ==0)
					{
						DEBUG("[Passive Lane change scenario, rht change]:front no collision, check this seg left\r\n");
						if(Trajectory->pathLane[0].hdmapPathInfo[segNum].laneNum == 2 || Trajectory->pathLane[0].hdmapPathInfo[segNum].laneNum == 4)
							// 车辆变道前无障碍物，且本路段存在左侧车道，对左侧车道进行碰撞检测（即当前车辆的本车道远方）
						{
							segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, \
								collision_dist, Trajectory->pathLane[0].hdmapPathInfo[segNum].laneInfos[1], envModelInfo,\
								&obj_flag_mid, temp_position);
							if(obj_flag_mid != 0)
							{
								DEBUG("[Passive Lane change scenario lc_exist rig change]:there have a remote mid obj After LC...\r\n");
								coll_obj->obj_mid_flag = obj_flag_mid;
								coll_obj->x_m = temp_position[0];
								coll_obj->y_m = temp_position[1];
								coll_obj->s_m = temp_position[2];
								DEBUG("[Passive Lane change scenario lc_exist rig change]:coll_obj->x_m = %f\r\n", coll_obj->x_m);
								DEBUG("[Passive Lane change scenario lc_exist rig change]:coll_obj->y_m = %f\r\n", coll_obj->y_m);
								DEBUG("[Passive Lane change scenario lc_exist rig change]:coll_obj->s_m = %f\r\n", coll_obj->s_m);
							}
						}
					}
					// 当前右侧无障碍物, 检测本路段碰撞（即当前车辆的右侧侧车道远方）
					if(obj_flag_right==0)
					{
						DEBUG("[Passive Lane change scenario, rht change]:right no collision, check this seg ego\r\n");
						segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, \
							collision_dist, Trajectory->pathLane[0].hdmapPathInfo[segNum].laneInfos[0], envModelInfo,\
							&obj_flag_right, temp_position);

						if(obj_flag_right != 0)
						{
							DEBUG("[Passive Lane change scenario lc_exist rig change]:there have a remote right obj After LC...\r\n");
							coll_obj->obj_rig_flag = obj_flag_right;
							coll_obj->x_r = temp_position[0];
							coll_obj->y_r = temp_position[1];
							coll_obj->s_r = temp_position[2];
							DEBUG("[Passive Lane change scenario lc_exist rig change]:coll_obj->x_r = %f\r\n", coll_obj->x_r);
							DEBUG("[Passive Lane change scenario lc_exist rig change]:coll_obj->y_r = %f\r\n", coll_obj->y_r);
							DEBUG("[Passive Lane change scenario lc_exist rig change]:coll_obj->s_r = %f\r\n", coll_obj->s_r);
						}
					}
					
				}
			}
		}	

	}
	else
	{
		// Local path 
		for (uint32 segNum = 0; segNum < Trajectory->localPath[0].segNum; segNum++)
		{
			DEBUG("Trajectory->localPath[0].hdmapPathInfo[0].laneInfos[0].nodeNum= %d\r\n", Trajectory->localPath[0].hdmapPathInfo[0].laneInfos[0].nodeNum);
			if (Trajectory->localPath[0].hdmapPathInfo[0].laneInfos[0].nodeNum <= 0)
			{
				//DEBUG("decision=hdmaperror \r\n");
				return;
			}

			segmentCollisionCheckWithDetail(veh_heading, veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, \
				collision_dist, Trajectory->localPath[0].hdmapPathInfo[segNum].laneInfos[0], envModelInfo, \
				obj_flag_mid_array, temp_position, cur_lane_width, decisionInfo);

			// if (obj_flag_mid != 0)  zlm 2021-0107 comment
			for(uint32 part_idx=0; part_idx<3; ++part_idx) //iterate each part of one lane
			{
				if(obj_flag_mid_array[part_idx]!=0)
				{
					if(part_idx==1)
					{
						DEBUG("===decision.cpp===there have a mid obj...\r\n");
						coll_obj->obj_mid_flag = obj_flag_mid_array[part_idx];
						coll_obj->x_m = temp_position[0];
						coll_obj->y_m = temp_position[1];
						coll_obj->s_m = temp_position[2];
						DEBUG("[===decision.cpp===]:coll_obj->x_m = %f\r\n", coll_obj->x_m);
						DEBUG("[===decision.cpp===]:coll_obj->y_m = %f\r\n", coll_obj->y_m);
						DEBUG("[===decision.cpp===]:coll_obj->s_m = %f\r\n", coll_obj->s_m);
					
					}
					if(part_idx==0)
					{
						DEBUG("===decision.cpp===there have a midLeft obj...\r\n");
						coll_obj->obj_midLeft_flag = obj_flag_mid_array[part_idx];
						coll_obj->x_m = temp_position[0];
						coll_obj->y_m = temp_position[1];
						coll_obj->s_m = temp_position[2];
						DEBUG("[===decision.cpp===]:coll_obj->x_m = %f\r\n", coll_obj->x_m);
						DEBUG("[===decision.cpp===]:coll_obj->y_m = %f\r\n", coll_obj->y_m);
						DEBUG("[===decision.cpp===]:coll_obj->s_m = %f\r\n", coll_obj->s_m);
					
					}
					if(part_idx==2)
					{
						DEBUG("===decision.cpp===there have a midRight obj...\r\n");
						coll_obj->obj_midRight_flag = obj_flag_mid_array[part_idx];
						coll_obj->x_m = temp_position[0];
						coll_obj->y_m = temp_position[1];
						coll_obj->s_m = temp_position[2];
						DEBUG("[===decision.cpp===]:coll_obj->x_m = %f\r\n", coll_obj->x_m);
						DEBUG("[===decision.cpp===]:coll_obj->y_m = %f\r\n", coll_obj->y_m);
						DEBUG("[===decision.cpp===]:coll_obj->s_m = %f\r\n", coll_obj->s_m);
						
					}
			    }
			}
			if(coll_obj->obj_mid_flag!=0) break; //current sgement make collision, quit iteration
		}

		//如果当前车道会发生碰撞，则判断左右车道障碍物
		// if (obj_flag_mid != 0)   // zlm comment, check side lane all the time 
		{
		for (uint32 segNum = 0; segNum < Trajectory->localPath[0].segNum; segNum++)
		{
			//判断当前段车道是否有左右车道
			DEBUG("Trajectory->localPath[0].hdmapPathInfo[segNum].laneNum = %d \r\n", Trajectory->localPath[0].hdmapPathInfo[segNum].laneNum);
			switch (Trajectory->localPath[0].hdmapPathInfo[segNum].laneNum)
			{
			case 2: //只有左车道
				if (obj_flag_left == 0)
				{
					segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, Trajectory->localPath[0].hdmapPathInfo[segNum].laneInfos[1], envModelInfo, &obj_flag_left, temp_position);
					if (obj_flag_left != 0)
					{
						DEBUG("===decision.cpp===there have a left obj...\r\n");
						coll_obj->obj_lef_flag = obj_flag_left;
						coll_obj->x_l = temp_position[0];
						coll_obj->y_l = temp_position[1];
						coll_obj->s_l = temp_position[2];
						DEBUG("[===decision.cpp===]:coll_obj->x_l = %f\r\n", coll_obj->x_l);
						DEBUG("[===decision.cpp===]:coll_obj->y_l = %f\r\n", coll_obj->y_l);
						DEBUG("[===decision.cpp===]:coll_obj->s_l = %f\r\n", coll_obj->s_l);
					}
				}
				break;
			case 3: //只有右车道
				if (obj_flag_right == 0)
				{
					segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, Trajectory->localPath[0].hdmapPathInfo[segNum].laneInfos[2], envModelInfo, &obj_flag_right, temp_position);
					if (obj_flag_right != 0)
					{
						DEBUG("===decision.cpp===there have a right obj...\r\n");
						coll_obj->obj_rig_flag = obj_flag_right;
						coll_obj->x_r = temp_position[0];
						coll_obj->y_r = temp_position[1];
						coll_obj->s_r = temp_position[2];
						DEBUG("[===decision.cpp===]:coll_obj->x_r = %f\r\n", coll_obj->x_r);
						DEBUG("[===decision.cpp===]:coll_obj->y_r = %f\r\n", coll_obj->y_r);
						DEBUG("[===decision.cpp===]:coll_obj->s_r = %f\r\n", coll_obj->s_r);
					}
				}
				break;
			case 4: //左右车道均有
				if (obj_flag_left == 0)
				{
					segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, Trajectory->localPath[0].hdmapPathInfo[segNum].laneInfos[1], envModelInfo, &obj_flag_left, temp_position);
					if (obj_flag_left != 0)
					{
						DEBUG("===decision.cpp===there have a left obj...\r\n");
						coll_obj->obj_lef_flag = obj_flag_left;
						coll_obj->x_l = temp_position[0];
						coll_obj->y_l = temp_position[1];
						coll_obj->s_l = temp_position[2];
						DEBUG("[===decision.cpp===]:coll_obj->x_l = %f\r\n", coll_obj->x_l);
						DEBUG("[===decision.cpp===]:coll_obj->y_l = %f\r\n", coll_obj->y_l);
						DEBUG("[===decision.cpp===]:coll_obj->s_l = %f\r\n", coll_obj->s_l);
					}
				}
				if (obj_flag_right == 0)
				{
					segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, Trajectory->localPath[0].hdmapPathInfo[segNum].laneInfos[2], envModelInfo, &obj_flag_right, temp_position);
					if (obj_flag_right != 0)
					{
						DEBUG("===decision.cpp===there have a right obj...\r\n");
						coll_obj->obj_rig_flag = obj_flag_right;
						coll_obj->x_r = temp_position[0];
						coll_obj->y_r = temp_position[1];
						coll_obj->s_r = temp_position[2];
						DEBUG("[===decision.cpp===]:coll_obj->x_r = %f\r\n", coll_obj->x_r);
						DEBUG("[===decision.cpp===]:coll_obj->y_r = %f\r\n", coll_obj->y_r);
						DEBUG("[===decision.cpp===]:coll_obj->s_r = %f\r\n", coll_obj->s_r);
					}
				}
				break;
			default:
				break;
			}
			if (coll_obj->obj_lef_flag > 0 && coll_obj->obj_rig_flag > 0)
			{
				break;
			}
		}
		}

		//////////////////////////////////////
		// 被动变道轨迹进行处理   zlm  全局路径
		//////////////////////////////////////
		
		// 被动变道场景检测 
		// DEBUG("[passive lane change scenario Begin],basicInfo-> globePLane->plan_seg_count = %d \r\n", globePLane->plan_seg_count);
				
		for (uint32 segNum = 0; segNum < globePLane->plan_seg_count; segNum++)
		{
			if(globePLane->PlanSeg[segNum].Lane[0].change_lane_flag ==1) // 存在被动变道的路段
			{
				
				if (fabs(float(globePLane->PlanSeg[segNum].Lane[0].lane_NO)) < \
					fabs(float(globePLane->PlanSeg[segNum+1].Lane[0].lane_NO)))
				lc_exist_type =1; // 向右侧变道
				lc_exist_flag =1;
				lc_exist_segnum = segNum;
				DEBUG("[passive lane change scenario],basicInfo-> lc_exist_flag =%d , lc_exist_segnum= %d，lc_exist_type= %d\r\n", 
					lc_exist_flag, lc_exist_segnum, lc_exist_type);
			}
		}
		// 被动变道场景 并且前方发生碰撞
		if(lc_exist_flag == 1 && obj_flag_mid!=0)
		{
			DEBUG("[passive lane change scenario] now lc_exist_flag == 1 && obj_flag_mid!=0 \r\n");
			//变道前的碰撞检测与处理

			obj_flag_mid = 0; //状态清理
			obj_flag_left = 0; 
			obj_flag_right = 0; 
			

			for (uint32 segNum = 0; segNum <= lc_exist_segnum; segNum++)
			{
				//对本车道进行二次碰撞检测 
				if(obj_flag_mid == 0) 
					segmentCollisionCheckWithDetail(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, \
						collision_dist, Trajectory->localPath[0].hdmapPathInfo[segNum].laneInfos[0], envModelInfo,\
					 	obj_flag_mid_array, temp_position, cur_lane_width, decisionInfo);
			}
			if(obj_flag_mid==0)   // 本车道处理
			{
				coll_obj->obj_mid_flag = 0;
				coll_obj->x_m = 0;
				coll_obj->y_m = 0;
				coll_obj->s_m = 0;
				DEBUG("[passive lane change scenario] second mid lane check, no collison...\r\n");
				// break;
			}
			for (uint32 segNum = 0; segNum <= lc_exist_segnum; segNum++) //相邻车道处理
			{
				//对相邻车道进行碰撞检测
				DEBUG("[Passive Lane change scenario, deal lc_exist_seg nearby lanes] : Trajectory->localPath[0].hdmapPathInfo[segNum].laneNum = %d \r\n", \
					Trajectory->localPath[0].hdmapPathInfo[segNum].laneNum);
				switch (Trajectory->localPath[0].hdmapPathInfo[segNum].laneNum)
				{
					case 2: //只有左车道
						if (obj_flag_left == 0)
						{
							segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, Trajectory->pathLane[0].hdmapPathInfo[segNum].laneInfos[1], envModelInfo, &obj_flag_left, temp_position);
							if (obj_flag_left != 0)
							{
								DEBUG("[Passive Lane change scenario case2]:there have a left obj befor LC...\r\n");
								
								coll_obj->obj_lef_flag = obj_flag_left;
								coll_obj->x_l = temp_position[0];
								coll_obj->y_l = temp_position[1];
								coll_obj->s_l = temp_position[2];
								DEBUG("[Passive Lane change scenario lc_exist case2]:coll_obj->x_l = %f\r\n",coll_obj->x_l);
								DEBUG("[Passive Lane change scenario lc_exist case2]:coll_obj->y_l = %f\r\n", coll_obj->y_l);
								DEBUG("[Passive Lane change scenario lc_exist case2]:coll_obj->s_l = %f\r\n",coll_obj->s_l);
							}
						}
						break;
					case 3: //只有右车道
						if (obj_flag_right == 0)
						{
							segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, Trajectory->localPath[0].hdmapPathInfo[segNum].laneInfos[2], envModelInfo, &obj_flag_right, temp_position);
							if (obj_flag_right != 0)
							{
								DEBUG("[Passive Lane change scenario case3]:there have a right obj befor LC...\r\n");
								coll_obj->obj_rig_flag = obj_flag_right;
								coll_obj->x_r = temp_position[0];
								coll_obj->y_r = temp_position[1];
								coll_obj->s_r = temp_position[2];
								DEBUG("[Passive Lane change scenario lc_exist case3]:coll_obj->x_r = %f\r\n",coll_obj->x_r);
								DEBUG("[Passive Lane change scenario lc_exist case3]:coll_obj->y_r = %f\r\n", coll_obj->y_r);
								DEBUG("[Passive Lane change scenario lc_exist case3]:coll_obj->s_r = %f\r\n",coll_obj->s_r);
							}
						}
						break;
					case 4: //左右车道均有
						if (obj_flag_left == 0)
						{
							segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, Trajectory->localPath[0].hdmapPathInfo[segNum].laneInfos[1], envModelInfo, &obj_flag_left, temp_position);
							if (obj_flag_left != 0)
							{
								DEBUG("[Passive Lane change scenario case 4]:there have a left obj befor LC...\r\n");
								coll_obj->obj_lef_flag = obj_flag_left;
								coll_obj->x_l = temp_position[0];
								coll_obj->y_l = temp_position[1];
								coll_obj->s_l = temp_position[2];
								DEBUG("[Passive Lane change scenario lc_exist case4]:coll_obj->x_l = %f\r\n", coll_obj->x_l);
								DEBUG("[Passive Lane change scenario lc_exist case4]:coll_obj->y_l = %f\r\n", coll_obj->y_l);
								DEBUG("[Passive Lane change scenario lc_exist case4]:coll_obj->s_l = %f\r\n", coll_obj->s_l);
							}
						}
						if (obj_flag_right == 0)
						{
							segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, Trajectory->localPath[0].hdmapPathInfo[segNum].laneInfos[2], envModelInfo, &obj_flag_right, temp_position);
							if (obj_flag_right != 0)
							{
								DEBUG("[Passive Lane change scenario case4]:there have a right obj befor LC...\r\n");
								coll_obj->obj_rig_flag = obj_flag_right;
								coll_obj->x_r = temp_position[0];
								coll_obj->y_r = temp_position[1];
								coll_obj->s_r = temp_position[2];
								DEBUG("[Passive Lane change scenario lc_exist case4]:coll_obj->x_r = %f\r\n", coll_obj->x_r);
								DEBUG("[Passive Lane change scenario lc_exist case4]:coll_obj->y_r = %f\r\n", coll_obj->y_r);
								DEBUG("[Passive Lane change scenario lc_exist case4]:coll_obj->s_r = %f\r\n", coll_obj->s_r);
							}
						}
						break;
					default:
						break;
				}
			}

			//变道后碰撞检测与处理
			DEBUG("[Passive Lane change scenario]:Trajectory->localPath[0].segNum = %d \r\n", Trajectory->localPath[0].segNum);
			for (uint32 segNum = lc_exist_segnum + 2; segNum < Trajectory->localPath[0].segNum; segNum++)
			{
				if(lc_exist_type==0)//当前左变道
				{
					if(obj_flag_mid ==0)
					{
								DEBUG("[Passive Lane change scenario, lf change]:front no collision, check this seg right\r\n");
						if(Trajectory->localPath[0].hdmapPathInfo[segNum].laneNum == 3 || Trajectory->localPath[0].hdmapPathInfo[segNum].laneNum == 4)
							// 车辆变道前无障碍物，且本路段存在右侧车道，对右侧车道进行碰撞检测（即当前车辆的本车道远方）
						{
							segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, \
								collision_dist, Trajectory->localPath[0].hdmapPathInfo[segNum].laneInfos[2], envModelInfo,\
								&obj_flag_mid, temp_position);
							if(obj_flag_mid != 0)
							{
								DEBUG("[Passive Lane change scenario, lf change]:there have a remote mid obj After LC...\r\n");
								coll_obj->obj_mid_flag = obj_flag_mid;
								coll_obj->x_m = temp_position[0];
								coll_obj->y_m = temp_position[1];
								coll_obj->s_m = temp_position[2];
								DEBUG("[Passive Lane change scenario lc_exist lf change]:coll_obj->x_m = %f\r\n", coll_obj->x_m);
								DEBUG("[Passive Lane change scenario lc_exist lf change]:coll_obj->y_m = %f\r\n", coll_obj->y_m);
								DEBUG("[Passive Lane change scenario lc_exist lf change]:coll_obj->s_m = %f\r\n", coll_obj->s_m);
							}
						}
					}
					// 检测本路段碰撞（即当前车辆的左侧车道远方）
					if(obj_flag_left==0)
					{
							DEBUG("[Passive Lane change scenario, lf change]:current left no collsion, check this seg ego\r\n");
							segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, \
							collision_dist, Trajectory->localPath[0].hdmapPathInfo[segNum].laneInfos[0], envModelInfo,\
							&obj_flag_left, temp_position);
						if(obj_flag_left != 0)
						{
							DEBUG("[Passive Lane change scenario lf change]:there have a remote left obj After LC...\r\n");
							coll_obj->obj_lef_flag = obj_flag_left;
							coll_obj->x_l = temp_position[0];
							coll_obj->y_l = temp_position[1];
							coll_obj->s_l = temp_position[2];
							DEBUG("[Passive Lane change scenario lc_exist lf change]:coll_obj->x_l = %f\r\n", coll_obj->x_l);
							DEBUG("[Passive Lane change scenario lc_exist lf change]:coll_obj->y_l = %f\r\n", coll_obj->y_l);
							DEBUG("[Passive Lane change scenario lc_exist lf change]:coll_obj->s_l = %f\r\n", coll_obj->s_l);
						}
					}
					
				}
				if(lc_exist_type==1) //当前为右变道
				{
					if(obj_flag_mid ==0)
					{
						DEBUG("[Passive Lane change scenario, rht change]:front no collision, check this seg left\r\n");
						if(Trajectory->localPath[0].hdmapPathInfo[segNum].laneNum == 2 || Trajectory->localPath[0].hdmapPathInfo[segNum].laneNum == 4)
							// 车辆变道前无障碍物，且本路段存在左侧车道，对左侧车道进行碰撞检测（即当前车辆的本车道远方）
						{
							segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, \
								collision_dist, Trajectory->localPath[0].hdmapPathInfo[segNum].laneInfos[1], envModelInfo,\
								&obj_flag_mid, temp_position);
							if(obj_flag_mid != 0)
							{
								DEBUG("[Passive Lane change scenario lc_exist rig change]:there have a remote mid obj After LC...\r\n");
								coll_obj->obj_mid_flag = obj_flag_mid;
								coll_obj->x_m = temp_position[0];
								coll_obj->y_m = temp_position[1];
								coll_obj->s_m = temp_position[2];
								DEBUG("[Passive Lane change scenario lc_exist rig change]:coll_obj->x_m = %f\r\n", coll_obj->x_m);
								DEBUG("[Passive Lane change scenario lc_exist rig change]:coll_obj->y_m = %f\r\n", coll_obj->y_m);
								DEBUG("[Passive Lane change scenario lc_exist rig change]:coll_obj->s_m = %f\r\n", coll_obj->s_m);
							}
						}
					}
					// 当前右侧无障碍物, 检测本路段碰撞（即当前车辆的右侧侧车道远方）
					if(obj_flag_right==0)
					{
						DEBUG("[Passive Lane change scenario, rht change]:right no collision, check this seg ego\r\n");
						segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, \
							collision_dist, Trajectory->localPath[0].hdmapPathInfo[segNum].laneInfos[0], envModelInfo,\
							&obj_flag_right, temp_position);

						if(obj_flag_right != 0)
						{
							DEBUG("[Passive Lane change scenario lc_exist rig change]:there have a remote right obj After LC...\r\n");
							coll_obj->obj_rig_flag = obj_flag_right;
							coll_obj->x_r = temp_position[0];
							coll_obj->y_r = temp_position[1];
							coll_obj->s_r = temp_position[2];
							DEBUG("[Passive Lane change scenario lc_exist rig change]:coll_obj->x_r = %f\r\n", coll_obj->x_r);
							DEBUG("[Passive Lane change scenario lc_exist rig change]:coll_obj->y_r = %f\r\n", coll_obj->y_r);
							DEBUG("[Passive Lane change scenario lc_exist rig change]:coll_obj->s_r = %f\r\n", coll_obj->s_r);
						}
					}
					
				}
			}
		}	
	}
}
#endif

/**************************************************************************************************
Function:           
Description: 把车辆轮廓视为矩形并进行膨胀，检测膨胀后覆盖的栅格是否被占用，用于决策模块,区分动静态障碍物。
Calls:              
Called By:          
Table Accessed:     
Table Updated:      
Input: 车辆后轴中心在栅格图坐标系下坐标，车辆航向，车宽，后轴中心与车头距离，后轴中心与车尾距离，膨胀尺寸，栅格图
Output: 0-不发生碰撞, 1-发生碰撞  
Others: 20200617,wsy
		20201022,wsy，修改为检测有障碍物的栅格数大于一定阈值认为发生碰撞
**************************************************************************************************/
int StaticDecision::collisionCheckInGridMap(const double ego_x, const double ego_y, const double ego_yaw, const double veh_width,
	const double veh_rear_axel_to_head, const double veh_rear_axel_to_tail, const double collision_dist,
	const Dt_ARRAY_80000_ObstacleGridMap ObstacleGridMap)
{
	//车辆坐标系，向前为x正，向左为y正,坐标原点在后轴中心
	//栅格图原点位于左上角，向右为X轴正方向，向下为Y轴正方向。车辆后轴中心在栅格图坐标系下坐标：(10,40)
	//栅格定义：0-可行驶区域，1-静态障碍物，2-动态障碍物，3-不可见区域。
	//
	//		|---->X
	//		|
	//      Y
	//
	//					x
	//					|
	//		      y <---|
	//          p1*---------*p4
	//			  |			|
	//			  |			|
	//			  |			|
	//			  |			|
	//			  |			|
	//          p2*---------*p3
	//

	double temp_x1 = veh_rear_axel_to_head + collision_dist;
	double temp_y1 = veh_width / 2 + collision_dist;
	double temp_x2 = -veh_rear_axel_to_tail - collision_dist;
	double temp_y2 = veh_width / 2 + collision_dist;
	double temp_x3 = -veh_rear_axel_to_tail - collision_dist;
	double temp_y3 = -veh_width / 2 - collision_dist;
	double temp_x4 = veh_rear_axel_to_head + collision_dist;
	double temp_y4 = -veh_width / 2 - collision_dist;

	int LOC_OCCUPIED_GRIDS_THRESHOLD = 30;//栅格碰撞检测阈值

	//DEBUG("ego_x: %f, ego_y: %f, ego_yaw: %f\n", ego_x, ego_y, ego_yaw);

	double x1, y1 = 0;
	int grid_x1, grid_y1 = 0;
	int occupied_grid_num_static = 0, occupied_grid_num_dynamic = 0;

	for (double t_x = temp_x1; t_x >= temp_x2 - 0.1; t_x -= 0.2)  //跳一个格，减少循环次数
	{	
		for (double t_y = temp_y1; t_y >= temp_y4 - 0.1; t_y -= 0.2)
		{
			//ego_yaw为车体坐标系x轴相对于栅格图坐标系X轴的夹角，逆时针为正。
			//将遍历到的车辆覆盖点从当前车辆坐标系转换到栅格图坐标系下。
			//20201019,wsy, 修改为从p1点开始，即左上角到右下角遍历；间隔修改为0.2m，跳格检测，有风险，后期需要处理！！！
			x1 = (t_x * cos(ego_yaw)  + (-t_y) * sin(ego_yaw) + ego_x) * 10;
			y1 = ((-t_y) * cos(ego_yaw) - t_x * sin(ego_yaw) + ego_y) * 10;

			grid_x1 = ROUND(x1);
			grid_y1 = ROUND(y1);
			//DEBUG("Grid_point: (%d,%d), \n", grid_y1, grid_x1);

			// if (grid_x1 >= 0 && grid_x1 < 199 && grid_y1 >= 0 && grid_y1 < 355)   // zlm 2021 -0114 comment 
			if (grid_x1 >= 0 && grid_x1 < 199 && grid_y1 >= 0 && grid_y1 < 399)  

				//如果矩形中的点在地图之外，不进行判断，输出非碰撞；栅格图左上角编号为（0，0），右下角为（200，400）
				//车头所在的栅格Y坐标序号为(362,99)
			{
				//往前后左右四个方向扩展格子，如果有障碍物，则插入队尾
				int di[4] = {0,1,0,1};
				int dj[4] = {0,0,1,1};
				for (int index = 0; index < 4; index++)
				{
					int next_grid_y1 = grid_y1 + di[index];
					int next_grid_x1 = grid_x1 + dj[index];
					if (ObstacleGridMap[next_grid_y1][next_grid_x1] == 1)
					{
						DEBUG("static ObstacleGridMap[%d][%d] = %d\r\n", next_grid_y1, next_grid_x1,ObstacleGridMap[next_grid_y1][next_grid_x1]);
						occupied_grid_num_static ++;
					}
				#if 1
					else if(ObstacleGridMap[next_grid_y1][next_grid_x1] == 2)
					{
						DEBUG("dynamic ObstacleGridMap[%d][%d] = %d\r\n", next_grid_y1, next_grid_x1 ,ObstacleGridMap[next_grid_y1][next_grid_x1]);
						occupied_grid_num_dynamic ++;
					}
				#endif
				}

				if (occupied_grid_num_static > LOC_OCCUPIED_GRIDS_THRESHOLD)
				{
					DEBUG("collisionCheckInGridMap: occupied_grid_num_static = %d\r\n", occupied_grid_num_static);
					return 1;
				}
			#if 1
				if(occupied_grid_num_dynamic > LOC_OCCUPIED_GRIDS_THRESHOLD)
				{
					DEBUG("collisionCheckInGridMap: occupied_grid_num_dynamic = %d\r\n", occupied_grid_num_dynamic);
					return 2;
				}
			#endif
			}
		}
	}

	return 0;
}


/**************************************************************************************************
Function:           
Description: 把车辆轮廓视为矩形并进行膨胀，检测膨胀后覆盖的栅格是否被占用，用于决策模块,区分动静态障碍物。
Calls:              
Called By:          
Table Accessed:     
Table Updated:      
Input: 车辆后轴中心在栅格图坐标系下坐标，车辆航向，车宽，后轴中心与车头距离，后轴中心与车尾距离，膨胀尺寸，栅格图
Output: 0-不发生碰撞, 1-发生碰撞  
Others: 20210110,zlm, add Midleft and Midright detect range
		20200617,wsy
		20201022,wsy，修改为检测有障碍物的栅格数大于一定阈值认为发生碰撞
**************************************************************************************************/
std::vector<int> StaticDecision::collisionCheckInGridMapWithDetail(const double ego_x, const double ego_y, const double ego_yaw, double lane_width, const double veh_width,
	const double veh_rear_axel_to_head, const double veh_rear_axel_to_tail, const double collision_dist, const Dt_ARRAY_80000_ObstacleGridMap ObstacleGridMap, const std::vector<std::vector<double>> &detect_range)
{
	//车辆坐标系，向前为x正，向左为y正,坐标原点在后轴中心
	//栅格图原点位于左上角，向右为X轴正方向，向下为Y轴正方向。车辆后轴中心在栅格图坐标系下坐标：(10,40)
	//栅格定义：0-可行驶区域，1-静态障碍物，2-动态障碍物，3-不可见区域。
	//
	//		|---->X
	//		|
	//      Y
	//
	//					x
	//					|
	//		      y <---|
	//          p1*---------*p4
	//			  |			|
	//			  |			|
	//			  |			|
	//			  |			|
	//			  |			|
	//          p2*---------*p3
	//
	// DEBUG("ego_x, ego_y,  ego_yaw= %f  %f  %f\r\n", ego_x, ego_y,  ego_yaw);
	int LOC_OCCUPIED_GRIDS_THRESHOLD = 30;//栅格碰撞检测阈值
	std::vector<int> flag_array={0,0,0};// default flag array

	double temp_x1[3]; 
	double temp_y1[3];
	double temp_x2[3];
	double temp_y2[3];
	double temp_x3[3];
	double temp_y3[3];
	double temp_x4[3];
	double temp_y4[3];

	//  check boundary param of Left Part
	temp_x1[0] = veh_rear_axel_to_head + collision_dist;
	temp_y1[0] = lane_width/2;
	temp_x2[0] = -veh_rear_axel_to_tail - collision_dist;
	temp_y2[0] = lane_width/2;
	temp_x3[0] = -veh_rear_axel_to_tail - collision_dist;
	temp_y3[0] = veh_width / 2 + collision_dist;
	temp_x4[0] = veh_rear_axel_to_head + collision_dist;
	temp_y4[0] = veh_width / 2 + collision_dist;

	//  check boundary param of Mid Part
	temp_x1[1] = veh_rear_axel_to_head + collision_dist;
	temp_y1[1] = veh_width / 2 + collision_dist;
	temp_x2[1] = -veh_rear_axel_to_tail - collision_dist;
	temp_y2[1] = veh_width / 2 + collision_dist;
	temp_x3[1] = -veh_rear_axel_to_tail - collision_dist;
	temp_y3[1] = -veh_width / 2 - collision_dist;
	temp_x4[1] = veh_rear_axel_to_head + collision_dist;
	temp_y4[1] = -veh_width / 2 - collision_dist;

	//  check boundary param of Right Part
	temp_x1[2] = veh_rear_axel_to_head + collision_dist;
	temp_y1[2] = -veh_width / 2 - collision_dist;
	temp_x2[2] = -veh_rear_axel_to_tail - collision_dist;
	temp_y2[2] = -veh_width / 2 - collision_dist;
	temp_x3[2] = -veh_rear_axel_to_tail - collision_dist;
	temp_y3[2] = -lane_width/2;
	temp_x4[2] = veh_rear_axel_to_head + collision_dist;
	temp_y4[2] = -lane_width/2;
	

	//DEBUG("ego_x: %f, ego_y: %f, ego_yaw: %f\n", ego_x, ego_y, ego_yaw);

	for(int part_idx=0; part_idx <3; ++ part_idx) // iterate each part(left, mid, right) of one lane
	{
		double x1, y1 = 0;
		int grid_x1, grid_y1 = 0;
		int occupied_grid_num_static = 0, occupied_grid_num_dynamic = 0;

		for (double t_x = temp_x1[part_idx]; t_x >= temp_x2[part_idx] - 0.1; t_x -= 0.2)  //跳一个格，减少循环次数
		{	
			for (double t_y = temp_y1[part_idx]; t_y >= temp_y4[part_idx] - 0.1; t_y -= 0.2)
			{
				//ego_yaw为车体坐标系x轴相对于栅格图坐标系X轴的夹角，逆时针为正。
				//将遍历到的车辆覆盖点从当前车辆坐标系转换到栅格图坐标系下。
				//20201019,wsy, 修改为从p1点开始，即左上角到右下角遍历；间隔修改为0.2m，跳格检测，有风险，后期需要处理！！！
				x1 = (t_x * cos(ego_yaw)  + (-t_y) * sin(ego_yaw) + ego_x) * 10;
				y1 = ((-t_y) * cos(ego_yaw) - t_x * sin(ego_yaw) + ego_y) * 10;

				grid_x1 = ROUND(x1);
				grid_y1 = ROUND(y1);
				//DEBUG("Grid_point: (%d,%d), \n", grid_y1, grid_x1);

				if (grid_x1 >= detect_range[0][0] && grid_x1 < detect_range[0][1] && grid_y1 >=detect_range[1][0] && grid_y1 < detect_range[1][1])  
					//如果矩形中的点在地图之外，不进行判断，输出非碰撞；栅格图左上角编号为（0，0），右下角为（200，400）
					//车头所在的栅格Y坐标序号为(362,99)
				{
					//往前后左右四个方向扩展格子，如果有障碍物，则插入队尾
					int di[4] = {0,1,0,1};
					int dj[4] = {0,0,1,1};
					for (int index = 0; index < 4; index++)
					{
						int next_grid_y1 = grid_y1 + di[index];
						int next_grid_x1 = grid_x1 + dj[index];
						if (ObstacleGridMap[next_grid_y1][next_grid_x1] == 1)
						{
							DEBUG("static ObstacleGridMap[%d][%d] = %d\r\n", next_grid_y1, next_grid_x1,ObstacleGridMap[next_grid_y1][next_grid_x1]);
							occupied_grid_num_static ++;
						}
					#if 1
						else if(ObstacleGridMap[next_grid_y1][next_grid_x1] == 2)
						{
							DEBUG("dynamic ObstacleGridMap[%d][%d] = %d\r\n", next_grid_y1, next_grid_x1 ,ObstacleGridMap[next_grid_y1][next_grid_x1]);
							occupied_grid_num_dynamic ++;
						}
					#endif
					}

				}
			}
		}

		if (occupied_grid_num_static > LOC_OCCUPIED_GRIDS_THRESHOLD)
				{
					DEBUG("collisionCheckInGridMap: occupied_grid_PartIdx(%d)_num_static = %d\r\n", part_idx, occupied_grid_num_static);
					flag_array[part_idx] =1;
					continue;
				}
			#if 1
				if(occupied_grid_num_dynamic > LOC_OCCUPIED_GRIDS_THRESHOLD)
				{
					DEBUG("collisionCheckInGridMap: occupied_grid_PartIdx(%d)_num_dynamic = %d\r\n", part_idx, occupied_grid_num_dynamic);
					flag_array[part_idx] =2;
					continue;
				}
			#endif
	}// iterate each part(left, mid, right) of one lane
	return flag_array;
}