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
更新:    
 		V2.0 improve compute effi   2021 - 01 -21
***************************************************************************************************/
void StaticDecision::segmentCollisionCheckWithDetail(float veh_heading, float veh_width, float veh_rear_axel_to_head, \
	 float veh_rear_axel_to_tail, float collision_dist, laneInfo single_laneinfo, Dt_RECORD_EnvModelInfos *envModelInfo,\
	 std::vector<int> &coll_flag, float64 *position, const double &lane_width,  decision_info *decisionInfo)
{	
	for (uint32 nodenum = 0; nodenum < single_laneinfo.nodeNum; nodenum+=3) //隔30cmm进行检测
	{
		float LocDeltaHeading = 0;
		if (nodenum == 0)
		{
			LocDeltaHeading = 90;
		}
		else
		{
			LocDeltaHeading =  single_laneinfo.laneNodeInfos[nodenum].heading - veh_heading + 90; 
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
		DEBUG("Loc_ego_x, Loc_ego_y = %f  %f\r\n", Loc_ego_x, Loc_ego_y);
		coll_flag = StaticDecision::collisionCheckInGridMapWithDetail(Loc_ego_x, Loc_ego_y, LocDeltaHeading, lane_width, veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, envModelInfo->ObstacleGridMap);
   		
		for(uint32 part_idx =0; part_idx<3;++part_idx)
		{
			if(coll_flag[part_idx]!= 0 )
			{
				// DEBUG("single_laneinfo.laneNodeInfos[%d].x = %f, single_laneinfo.laneNodeInfos[%d].y = %f, single_laneinfo.laneNodeInfos[%d].s = %f \r\n", nodenum, single_laneinfo.laneNodeInfos[nodenum].x, nodenum, single_laneinfo.laneNodeInfos[nodenum].y, nodenum, single_laneinfo.laneNodeInfos[nodenum].s);
				position[0] = single_laneinfo.laneNodeInfos[nodenum].x;
				position[1] = single_laneinfo.laneNodeInfos[nodenum].y;
				position[2] = single_laneinfo.laneNodeInfos[nodenum].x + veh_rear_axel_to_head;//lkw,20201229,距离后轴中心距离
				DEBUG("part_idx= %d\r\n", part_idx);
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
* 功  能: coordinate convert help to convert grid to vehicle
* 输  入: vehicle size info, lane to check, obstacle grid info 
* 输  出: collsion flag , obstacle distance s

更新:    V2.0 add SideAvoid Logic    2021 - 01 -18
***************************************************************************************************/
void StaticDecision::segmentCollisionCheck(float veh_heading, float veh_width, float veh_rear_axel_to_head,\
	 float veh_rear_axel_to_tail, float collision_dist, laneInfo single_laneinfo, Dt_RECORD_EnvModelInfos *envModelInfo,\
	 int *coll_flag, float64 *position)
{
	for (uint32 nodenum = 0; nodenum < single_laneinfo.nodeNum; nodenum+=3) //隔30 cm进行检测
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
		// LocDeltaHeading = 90;
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

    
		*coll_flag = StaticDecision::collisionCheckInGridMap(Loc_ego_x, Loc_ego_y, LocDeltaHeading, veh_width,
														 veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, envModelInfo->ObstacleGridMap);
   		

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

/**
 * @brief a assist fun which help to assign value and output debug infos;
 * @param left value to be assigned
 * @param right value will assigned to others
 * @return nothing
 */
void AssignAndDebugGridInfo( const std::string &s2, gridmap_coll_obj &left,  float64 *right, const std::string &s1 =""){
    DEBUG("RouteObjDectbyGrid[%s] there is a obj in lane ->> %s \r\n", s1.c_str(), s2.c_str());
	// coll_obj->obj_mid_flag = obj_flag_mid_array[part_idx];
	left.x_m = right[0];
	left.y_m = right[1];
	left.s_m = right[2];
	DEBUG("RouteObjDectbyGrid :coll_obj->x_m = %f\r\n", left.x_m );
	DEBUG("RouteObjDectbyGrid :coll_obj->y_m = %f\r\n", left.y_m );
	DEBUG("RouteObjDectbyGrid :coll_obj->s_m = %f\r\n", left.s_m );
}


/**
 * @brief grid check Main Logic, which help to check interseted area at diff scenario
 * update:
 * 		V2.2 2021-02-02 optimized coding style
 * 		V2.1 2021-01-20 review code, and fix some format errors 
		V2.0 2021-01-18 add Side Avoid Logic 
 * @param envModelInfo environment infos
 * @param ... others
 * @return coll_obj  which indicator which position appear obj
 */
void StaticDecision::RouteObjDectbyGrid(float veh_heading, int onpath, hdMapTrajectory *Trajectory, Dt_RECORD_HdmapInfo *hdmapInfos,\
 		Dt_RECORD_EnvModelInfos *envModelInfo, gridmap_coll_obj *coll_obj, Dt_RECORD_HdmapFrontPLane *globePLane, Dt_RECORD_HdmapLocalLane *localPLanne,  decision_info *decisionInfo)
{
	float veh_width = 1.95;
	float veh_rear_axel_to_head = 3.75;
	float veh_rear_axel_to_tail = 0.95;
	float collision_dist = 0.2;

	std::vector<int> obj_flag_mid_array;   //zlm 2021-0107 add 障碍物标志位(Midleft Mid Midright)
	int obj_flag_mid = 0, obj_flag_left = 0, obj_flag_right = 0;
	float64 temp_position[3] = {0}; //0---x,1---y,2---dis;
	

	int lc_exist_flag = 0;// 轨迹中存在LC_flag的标志位
	uint32 lc_exist_segnum = 0;
	int lc_exist_type = 0; //0->left  1->right
	static double valid_lane_width;
	double cur_lane_width;

	// get valid lanewidth
	if(onpath)
		cur_lane_width = globePLane->PlanSeg[0].Lane[0].lane_width;
	else
		cur_lane_width = localPLanne->LocalLane[0].lane_width;

	if(cur_lane_width > 0) // save valid lane
		valid_lane_width = cur_lane_width;
	else 				   // read valid lane
		cur_lane_width = valid_lane_width;
	
	// get refpath
	DEBUG("RouteObjDectbyGrid : onpath = %d, decision_command = %d\r\n", onpath, decisionInfo->decision_command);
	hdmapPathInfos refpath_;
	if (onpath == TRUE || decisionInfo->decision_command == LEFTAVOID || decisionInfo->decision_command == RIGHTAVOID)//在全局轨迹上或者壁障中
	{
		refpath_ = Trajectory->pathLane[0];
	}
	else
	{
		refpath_ =Trajectory->localPath[0];
	}
	
	// check mid lane with all segments
	for (uint32 segNum = 0; segNum < refpath_.segNum; segNum++)
	{
		DEBUG("refpath_.hdmapPathInfo[segNum].laneInfos[0].nodeNum= %d\r\n", refpath_.hdmapPathInfo[segNum].laneInfos[0].nodeNum);
		if (refpath_.hdmapPathInfo[0].laneInfos[0].nodeNum <= 0)
		{
			// DEBUG("decision=hdmaperror \r\n");
			return;
		}
		// check ego lane with midleft mid midright
		segmentCollisionCheckWithDetail(veh_heading, veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, refpath_.hdmapPathInfo[segNum].laneInfos[0], envModelInfo,\
			obj_flag_mid_array, temp_position, cur_lane_width, decisionInfo);

		for(uint32 part_idx=0; part_idx<3; ++part_idx) //iterate each part of one lane
		{
			if(obj_flag_mid_array[part_idx]!=0)
			switch (part_idx)
			{
				case 0:
					coll_obj->obj_midLeft_flag = obj_flag_mid_array[part_idx];
					AssignAndDebugGridInfo("midLeft", *coll_obj, temp_position);
					break;
				case 1:
					coll_obj->obj_mid_flag = obj_flag_mid_array[part_idx];
					AssignAndDebugGridInfo("mid", *coll_obj, temp_position);
					break;
				case 2:
					coll_obj->obj_midRight_flag = obj_flag_mid_array[part_idx];
					AssignAndDebugGridInfo("midRight", *coll_obj, temp_position);
					break;
			}
		}
		if(coll_obj->obj_mid_flag!=0) break; //current sgement make collision, quit iteration
	}// check mid lane with all segments

	//判断左右车道障碍物
	for (uint32 segNum = 0; segNum < refpath_.segNum; segNum++)
	{
		//判断当前段车道是否有左右车道
		DEBUG("refpath_.hdmapPathInfo[segNum].laneNum = %d \r\n", refpath_.hdmapPathInfo[segNum].laneNum);
		switch (refpath_.hdmapPathInfo[segNum].laneNum)
		{
			case 2: //只有左车道
				if (obj_flag_left == 0)
				{
					segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, refpath_.hdmapPathInfo[segNum].laneInfos[1], envModelInfo, &obj_flag_left, temp_position);
					if (obj_flag_left != 0)
					{
						coll_obj->obj_lef_flag = obj_flag_left;
						AssignAndDebugGridInfo("left", *coll_obj, temp_position);
					}
				}
				break;
					
			case 3: //只有右车道
				if (obj_flag_right == 0)
				{
					segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, refpath_.hdmapPathInfo[segNum].laneInfos[2], envModelInfo, &obj_flag_right, temp_position);
					if (obj_flag_right != 0)
					{
						coll_obj->obj_rig_flag = obj_flag_right;
						AssignAndDebugGridInfo("right", *coll_obj, temp_position);
					}
				}
				break;
		
			case 4: //左右车道均有
				if (obj_flag_left == 0)
				{
					segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, refpath_.hdmapPathInfo[segNum].laneInfos[1], envModelInfo, &obj_flag_left, temp_position);
					if (obj_flag_left != 0)
					{
						coll_obj->obj_lef_flag = obj_flag_left;
						AssignAndDebugGridInfo("left", *coll_obj, temp_position);
					}
				}
				if (obj_flag_right == 0)
				{
					segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, refpath_.hdmapPathInfo[segNum].laneInfos[2], envModelInfo, &obj_flag_right, temp_position);
					if (obj_flag_right != 0)
					{
						coll_obj->obj_rig_flag = obj_flag_right;
						AssignAndDebugGridInfo("right", *coll_obj, temp_position);
					}
				}
				break;
			default:
				break;
		}// switch, check side lane
		if (coll_obj->obj_lef_flag > 0 && coll_obj->obj_rig_flag > 0)
			break;
	}//for, check all segments
	
	// 被动变道场景检测 	
	for (uint32 segNum = 0; segNum < globePLane->plan_seg_count; segNum++)
	{
		if(globePLane->PlanSeg[segNum].Lane[0].change_lane_flag ==1) // 存在被动变道的路段
		{
			
			if (fabs(float(globePLane->PlanSeg[segNum].Lane[0].lane_NO)) < fabs(float(globePLane->PlanSeg[segNum+1].Lane[0].lane_NO)))
			{
				lc_exist_type =1; // 向右侧变道
			}
			lc_exist_flag =1;
			lc_exist_segnum = segNum;
			DEBUG("[passive lane change scenario],basicInfo-> lc_exist_flag =%d , lc_exist_segnum= %d，lc_exist_type= %d\r\n", lc_exist_flag, lc_exist_segnum, lc_exist_type);
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
					collision_dist, refpath_.hdmapPathInfo[segNum].laneInfos[0], envModelInfo, obj_flag_mid_array, temp_position, cur_lane_width, decisionInfo);
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
		for (uint32 segNum = 0; segNum <= lc_exist_segnum; segNum++) //处理碰撞segment及其前路段相邻车道
		{
			//对相邻车道进行碰撞检测
			DEBUG("[Passive Lane change scenario, deal lc_exist_seg nearby lanes] : refpath_.hdmapPathInfo[segNum].laneNum = %d \r\n", \
				refpath_.hdmapPathInfo[segNum].laneNum);
			switch (refpath_.hdmapPathInfo[segNum].laneNum)
			{
				case 2: //只有左车道
					if (obj_flag_left == 0)
					{
						segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, refpath_.hdmapPathInfo[segNum].laneInfos[1], envModelInfo, &obj_flag_left, temp_position);
						if (obj_flag_left != 0)
						{								
							coll_obj->obj_lef_flag = obj_flag_left;
							AssignAndDebugGridInfo( "left befor LC", *coll_obj, temp_position, "Passive Lane change scenario case2");
						}
					}
					break;
				case 3: //只有右车道
					if (obj_flag_right == 0)
					{
						segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, refpath_.hdmapPathInfo[segNum].laneInfos[2], envModelInfo, &obj_flag_right, temp_position);
						if (obj_flag_right != 0)
						{
							coll_obj->obj_rig_flag = obj_flag_right;
							AssignAndDebugGridInfo( "right befor LC", *coll_obj, temp_position, "Passive Lane change scenario case3");
						}
					}
					break;
				case 4: //左右车道均有
					if (obj_flag_left == 0)
					{
						segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, refpath_.hdmapPathInfo[segNum].laneInfos[1], envModelInfo, &obj_flag_left, temp_position);
						if (obj_flag_left != 0)
						{
							DEBUG("[Passive Lane change scenario case 4]:there have a left obj befor LC...\r\n");
							coll_obj->obj_lef_flag = obj_flag_left;
							AssignAndDebugGridInfo( "left befor LC", *coll_obj, temp_position, "Passive Lane change scenario case4");
						}
					}
					if (obj_flag_right == 0)
					{
						segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, collision_dist, refpath_.hdmapPathInfo[segNum].laneInfos[2], envModelInfo, &obj_flag_right, temp_position);
						if (obj_flag_right != 0)
						{
							coll_obj->obj_rig_flag = obj_flag_right;
							AssignAndDebugGridInfo( "right befor LC", *coll_obj, temp_position, "Passive Lane change scenario case4");
						}
					}
					break;
				default:
					break;
			}
		}//处理碰撞segment及其前路段相邻车道

		//变道后碰撞检测与处理
		DEBUG("[Passive Lane change scenario]:refpath_.segNum = %d \r\n", refpath_.segNum);
		for (uint32 segNum = lc_exist_segnum + 2; segNum < refpath_.segNum; segNum++)
		{
			if(lc_exist_type==0)//当前左变道
			{
				if(obj_flag_mid ==0)
				{
							DEBUG("[Passive Lane change scenario, lf change]:front no collision, check this seg right\r\n");
					if(refpath_.hdmapPathInfo[segNum].laneNum == 3 || refpath_.hdmapPathInfo[segNum].laneNum == 4)
						// 车辆变道前无障碍物，且本路段存在右侧车道，对右侧车道进行碰撞检测（即当前车辆的本车道远方）
					{
						segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, \
							collision_dist, refpath_.hdmapPathInfo[segNum].laneInfos[2], envModelInfo,\
							&obj_flag_mid, temp_position);
						if(obj_flag_mid != 0)
						{
							DEBUG("[Passive Lane change scenario, lf change]:there have a remote mid obj After LC...\r\n");
							coll_obj->obj_mid_flag = obj_flag_mid;
							AssignAndDebugGridInfo( "remote mid After LC", *coll_obj, temp_position, "Passive Lane change scenario");
						}
					}
				}
				// 检测本路段碰撞（即当前车辆的左侧车道远方）
				if(obj_flag_left==0)
				{
						DEBUG("[Passive Lane change scenario, lf change]:current left no collsion, check this seg ego\r\n");
						segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, \
						collision_dist, refpath_.hdmapPathInfo[segNum].laneInfos[0], envModelInfo,\
						&obj_flag_left, temp_position);
					if(obj_flag_left != 0)
					{
						coll_obj->obj_lef_flag = obj_flag_left;
						AssignAndDebugGridInfo( "remote left After LC", *coll_obj, temp_position, "Passive Lane change scenario left change");
					}
				}
				
			}
			if(lc_exist_type==1) //当前为右变道
			{
				if(obj_flag_mid ==0)
				{
					DEBUG("[Passive Lane change scenario, rht change]:front no collision, check this seg left\r\n");
					if(refpath_.hdmapPathInfo[segNum].laneNum == 2 || refpath_.hdmapPathInfo[segNum].laneNum == 4)
						// 车辆变道前无障碍物，且本路段存在左侧车道，对左侧车道进行碰撞检测（即当前车辆的本车道远方）
					{
						segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, \
							collision_dist, refpath_.hdmapPathInfo[segNum].laneInfos[1], envModelInfo,\
							&obj_flag_mid, temp_position);
						if(obj_flag_mid != 0)
						{
							DEBUG("[Passive Lane change scenario lc_exist rig change]:there have a remote mid obj After LC...\r\n");
							coll_obj->obj_mid_flag = obj_flag_mid;
							AssignAndDebugGridInfo( "remote mid obj After LC", *coll_obj, temp_position, "Passive Lane change scenario lc_exist right change");
						}
					}
				}
				// 当前右侧无障碍物, 检测本路段碰撞（即当前车辆的右侧侧车道远方）
				if(obj_flag_right==0)
				{
					DEBUG("[Passive Lane change scenario, rht change]:right no collision, check this seg ego\r\n");
					segmentCollisionCheck(veh_heading,veh_width, veh_rear_axel_to_head, veh_rear_axel_to_tail, \
						collision_dist, refpath_.hdmapPathInfo[segNum].laneInfos[0], envModelInfo,\
						&obj_flag_right, temp_position);

					if(obj_flag_right != 0)
					{
						coll_obj->obj_rig_flag = obj_flag_right;
						AssignAndDebugGridInfo( "remote right obj After LC", *coll_obj, temp_position, "Passive Lane change scenario lc_exist right change");
					}
				}
			}
		}//变道后碰撞检测与处理
	}// 被动变道场景 并且前方发生碰撞
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

	int LOC_OCCUPIED_GRIDS_THRESHOLD_STATIC  = 30;//栅格碰撞检测阈值
	int LOC_OCCUPIED_GRIDS_THRESHOLD_DYNAMIC = 20;//栅格碰撞检测阈值


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

				if (occupied_grid_num_static > LOC_OCCUPIED_GRIDS_THRESHOLD_STATIC)
				{
					DEBUG("collisionCheckInGridMap: occupied_grid_num_static = %d\r\n", occupied_grid_num_static);
					return 1;
				}
			#if 1
				if(occupied_grid_num_dynamic > LOC_OCCUPIED_GRIDS_THRESHOLD_DYNAMIC)
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
Others: 20210121,zlm, restrict dectet range which help to exclude ego car's body miss recognize
		20210110,zlm, add Midleft and Midright detect range
		20200617,wsy
		20201022,wsy，修改为检测有障碍物的栅格数大于一定阈值认为发生碰撞
**************************************************************************************************/
std::vector<int> StaticDecision::collisionCheckInGridMapWithDetail(const double ego_x, const double ego_y, const double ego_yaw, double lane_width, const double veh_width,
	const double veh_rear_axel_to_head, const double veh_rear_axel_to_tail, const double collision_dist, const Dt_ARRAY_80000_ObstacleGridMap ObstacleGridMap)
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
	std::vector<int> flag_array={0,0,0};// default flag array (Midleft Mid MidRight)

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
	temp_y1[0] = lane_width/2 - collision_dist;
	temp_x2[0] = -veh_rear_axel_to_tail - collision_dist;
	temp_y2[0] = lane_width/2 - collision_dist;
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
	temp_y3[2] = -lane_width /2+ collision_dist;
	temp_x4[2] = veh_rear_axel_to_head + collision_dist;
	temp_y4[2] = -lane_width /2+ collision_dist;
	

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

				//如果矩形中的点在地图之外，不进行判断，输出非碰撞；栅格图左上角编号为（0，0），右下角为（200，400）
					//车头所在的栅格Y坐标序号为(362,99)
				if (grid_x1 >= 0 && grid_x1 < 199 && grid_y1 >= 0 && grid_y1 < 355)  
				{
					// if(grid_x1 >= 100 - 12 && grid_x1 < 100 + 12 && grid_y1 >= 355 && grid_y1 < 399) // exclude ego vehicle range
					// continue;
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