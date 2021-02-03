#include "../libs/obstacle_selection.h"
/***************************************************************************************************
* 功  能: 对地图数据进行处理，坐标系转换及接口标准化
* 输  入:Dt_RECORD_HdmapInfo* hdmapInfos 全局路径终点坐标在地图坐标系下的位置
Dt_RECORD_HdmapFrontPLane* globePLane当在全局路径上时，地图规划的全局路径及左右相邻车道在地图坐标系中的信息
Dt_RECORD_HdmapLocalLane* hdMap 当不在全局路径上时，地图当前车道、下一段车道及左右相邻车道在地图坐标系中的信息
double x_veh 车辆在地图坐标系下的x坐标
double y_veh 车辆在地图坐标系下的y坐标
double heading 车辆航向
* 输  出: Trajectory地图发来的轨迹点在车体坐标系下的坐标
hdmapInfos 地图中的全局路径终点信息转换成了车体坐标系下坐标
缺少对后续轨迹的处理，后期会增加
调用函数：int getGridCoordiFromGlobalXY(double heading, double x_veh, double y_veh, double x_pt, double y_pt, double* x_out, double* y_out)全局坐标系向车辆坐标系转换的函数
***************************************************************************************************/
void decision::DealWithHdmap(Dt_RECORD_HdmapInfo *hdmapInfos, double xVeh, double yVeh, double heading, Dt_RECORD_HdmapFrontPLane *globePLane, Dt_RECORD_HdmapLocalLane *localPLanne, hdMapTrajectory *Trajectory)
{
	//无论是否在全局路径上，都需要对全局轨迹上的信息进行处理
	/*在全局路径上对轨迹进行坐标系转换及稀疏处理*/
	Trajectory->pathLane[0].segNum = globePLane->plan_seg_count;
	for (uint32 count = 0; count < globePLane->plan_seg_count; count++)
	{
		if ((globePLane->PlanSeg[count].Lane[0].node_count > 100) || (globePLane->PlanSeg[count].Lane[0].node_count == 0))
		{
			std::cout <<"TEST:::" <<(double)globePLane->PlanSeg[count].Lane[0].node_count;
			DEBUG("PlanSegInfoerror \r\n");
			return;
		}
	}
	/*对前向轨迹进行处理：i表示车道段，确定有多少个车道段，进行循环处理*/
	for (uint32 i = 0; i < globePLane->plan_seg_count; i++)
	{
		/*表示当前车道是否有左右相邻车道*/
		Trajectory->pathLane[0].hdmapPathInfo[i].laneNum = globePLane->PlanSeg[i].lane_count;
		DEBUG("Trajectory->pathLane[0].hdmapPathInfo[%d].laneNum = %d \r\n", i, Trajectory->pathLane[0].hdmapPathInfo[i].laneNum);

		/*对当前全局路径进行处理*/
		double dis = 0;
		uint32 nodecount = 0;
#if 1
		if (i == 0)
		{
			dis = 0;
		}
		else if (i >= 2)
		{
			//判断上一段是否为换道区间
			if (globePLane->PlanSeg[i - 1].Lane[0].change_lane_flag == 1)
			{
				//距离初始是从上上段的距离进行累加
				nodecount = Trajectory->pathLane[0].hdmapPathInfo[i - 2].laneInfos[0].nodeNum;
				if (nodecount > 0)
				{
					dis = Trajectory->pathLane[0].hdmapPathInfo[i - 2].laneInfos[0].laneNodeInfos[nodecount - 1].s;
				}
				else
				{
					dis = 0;
				}
			}
			else
			{
				nodecount = Trajectory->pathLane[0].hdmapPathInfo[i - 1].laneInfos[0].nodeNum;
				if (nodecount > 0)
				{
					dis = Trajectory->pathLane[0].hdmapPathInfo[i - 1].laneInfos[0].laneNodeInfos[nodecount - 1].s;
				}
				else
				{
					dis = 0;
				}
			}
		}
		else
		{
			if (globePLane->PlanSeg[i - 1].Lane[0].change_lane_flag == 1)
			{
				dis = 0;
			}
			else
			{
				nodecount = Trajectory->pathLane[0].hdmapPathInfo[i - 1].laneInfos[0].nodeNum;
				if (nodecount > 0)
				{
					dis = Trajectory->pathLane[0].hdmapPathInfo[i - 1].laneInfos[0].laneNodeInfos[nodecount - 1].s;
				}
				else
				{
					dis = 0;
				}
			}
		}
#endif
		DealWithNode(dis, &globePLane->PlanSeg[i].Lane[0], &Trajectory->pathLane[0].hdmapPathInfo[i].laneInfos[0], xVeh, yVeh, heading, hdmapInfos->origin_yaw);
		// DEBUG("===DealWithHdmap===Trajectory->pathLane[0].hdmapPathInfo[%d].laneInfos[0] = %d\r\n", i, Trajectory->pathLane[0].hdmapPathInfo[i].laneInfos[0].laneNodeInfos.size());
		/*若有左车道或左右车道均存在，则对左车道轨迹点进行坐标转换*/
		if (globePLane->PlanSeg[i].lane_count == 2 || globePLane->PlanSeg[i].lane_count == 4)
		{
			if ((globePLane->PlanSeg[i].Lane[1].node_count > 100) || (globePLane->PlanSeg[i].Lane[1].node_count == 0))
			{
				DEBUG("PlanSegLeftInfoerror \r\n");
				return;
			}
			DealWithNode(dis, &globePLane->PlanSeg[i].Lane[1], &Trajectory->pathLane[0].hdmapPathInfo[i].laneInfos[1], xVeh, yVeh, heading, hdmapInfos->origin_yaw);
		}
		/*若有右车道或左右车道均存在，则对右车道轨迹点进行坐标转换*/
		if ((globePLane->PlanSeg[i].lane_count == 3) || (globePLane->PlanSeg[i].lane_count == 4))
		{
			if ((globePLane->PlanSeg[i].Lane[2].node_count > 100) || (globePLane->PlanSeg[i].Lane[2].node_count == 0))
			{
				DEBUG("PlanSegRihhtInfoerror \r\n");
				return;
			}
			DealWithNode(dis, &globePLane->PlanSeg[i].Lane[2], &Trajectory->pathLane[0].hdmapPathInfo[i].laneInfos[2], xVeh, yVeh, heading, hdmapInfos->origin_yaw);
		}
	}
	/*对后向轨迹进行处理*/
	/*	for ()
		{
		}*/
	if (hdmapInfos->planpath == false)
	{
		/*若不在全局轨迹上，对当前所在车道轨迹进行处理*/
		Trajectory->localPath[0].hdmapPathInfo[0].laneNum = localPLanne->lane_count;
		Trajectory->localPath[0].segNum = localPLanne->next_seg_count + 1;
		DEBUG("===DealWithHdmap===localPLanne->next_seg_count = %d\r\n", localPLanne->next_seg_count);
		DEBUG("===DealWithHdmap===localPLanne->lane_count = %d\r\n", localPLanne->lane_count);
		if ((localPLanne->LocalLane[0].node_count > 100) || (localPLanne->LocalLane[0].node_count == 0))
		{
			DEBUG("LocalLaneInfoerror \r\n");
			return;
		}
		DealWithNode(0, &localPLanne->LocalLane[0], &Trajectory->localPath[0].hdmapPathInfo[0].laneInfos[0], xVeh, yVeh, heading, hdmapInfos->origin_yaw);
		if (localPLanne->lane_count == 2 || localPLanne->lane_count == 4)
		{
			if ((localPLanne->LocalLane[1].node_count > 100) || (localPLanne->LocalLane[1].node_count == 0))
			{
				DEBUG("LocalLaneLeftInfoerror \r\n");
				return;
			}
			DealWithNode(0, &localPLanne->LocalLane[1], &Trajectory->localPath[0].hdmapPathInfo[0].laneInfos[1], xVeh, yVeh, heading, hdmapInfos->origin_yaw);
		}
		if (localPLanne->lane_count == 3 || localPLanne->lane_count == 4)
		{
			if ((localPLanne->LocalLane[2].node_count > 100) || (localPLanne->LocalLane[2].node_count == 0))
			{

				DEBUG("LocalRightInfoerror \r\n");
				return;
			}
			DealWithNode(0, &localPLanne->LocalLane[2], &Trajectory->localPath[0].hdmapPathInfo[0].laneInfos[2], xVeh, yVeh, heading, hdmapInfos->origin_yaw);
		}
		/*若不在全局轨迹上，对当前车道的下一段车道轨迹进行处理*/

		for (uint32 j = 0; j < localPLanne->next_seg_count; j++)
		{
			DEBUG("===DealWithHdmap===localPLanne->next_seg_count = %d\r\n", localPLanne->next_seg_count);
			uint32 Loc_NodeNum = Trajectory->localPath[0].hdmapPathInfo[0].laneInfos[0].nodeNum;
			double Loc_Dis = 0;
			if (Loc_NodeNum > 1)
			{
				Loc_Dis = Trajectory->localPath[0].hdmapPathInfo[0].laneInfos[0].laneNodeInfos[Loc_NodeNum - 1].s;
			}
			Trajectory->localPath[0].hdmapPathInfo[j + 1].laneNum = localPLanne->lane_count;
			if ((localPLanne->NextSeg[j].Lane[0].node_count > 100) || (localPLanne->NextSeg[j].Lane[0].node_count == 0))
			{
				DEBUG("NextSegInfoerror \r\n");
				return;
			}
			DealWithNode(Loc_Dis, &localPLanne->NextSeg[j].Lane[0], &Trajectory->localPath[0].hdmapPathInfo[j + 1].laneInfos[0], xVeh, yVeh, heading, hdmapInfos->origin_yaw);

			if (localPLanne->NextSeg[j].lane_count == 2 || localPLanne->NextSeg[j].lane_count == 4)
			{
				if ((localPLanne->NextSeg[j].Lane[1].node_count > 100) || (localPLanne->NextSeg[j].Lane[1].node_count == 0))
				{
					DEBUG("NextSegLeftInfoerror \r\n");
					return;
				}
				DealWithNode(Loc_Dis, &localPLanne->NextSeg[j].Lane[1], &Trajectory->localPath[0].hdmapPathInfo[j + 1].laneInfos[1], xVeh, yVeh, heading, hdmapInfos->origin_yaw);
			}
			if (localPLanne->NextSeg[j].lane_count == 3 || localPLanne->NextSeg[j].lane_count == 4)
			{
				if ((localPLanne->NextSeg[j].Lane[2].node_count > 100) || (localPLanne->NextSeg[j].Lane[2].node_count == 0))
				{

					DEBUG("NextSegRightInfoerror \r\n");
					return;
				}

				DealWithNode(Loc_Dis, &localPLanne->NextSeg[j].Lane[2], &Trajectory->localPath[0].hdmapPathInfo[j + 1].laneInfos[2], xVeh, yVeh, heading, hdmapInfos->origin_yaw);
			}
		}
	}
}
/***************************************************************************************************
* 功  能: 对全局轨迹坐标点进行插值并进行坐标转换
* 输  入:     Dt_RECORD_FrontLane* frontLane   来自地图的轨迹点
* 输  出: laneInfo *outglobalTraj; 转换后的坐标点
未经过测试，目前暂定不变
***************************************************************************************************/
void decision::DealWithNode(double dis, Dt_RECORD_HdMapLane *frontLane, laneInfo *globalTraj, double xVeh, double yVeh, double heading, double mapHeading)
{
	laneInfo outglobalTraj;
	memset(&outglobalTraj, 0, sizeof(laneInfo));
	int nodeNum = 0;
	/*将当前车道坐标点进行插值*/
	DEBUG("===DealWithNode===frontLan->node_count = %d\r\n", frontLane->node_count);
	for (uint32 k = 1; k < frontLane->node_count; k++)
	{

		double xs = frontLane->LaneNode[k - 1].hdmap_x;
		double ys = frontLane->LaneNode[k - 1].hdmap_y;
		double cs = frontLane->LaneNode[k - 1].curvature;
		double hs = frontLane->LaneNode[k - 1].heading;
		double xt = frontLane->LaneNode[k].hdmap_x;
		double yt = frontLane->LaneNode[k].hdmap_y;
		double ct = frontLane->LaneNode[k].curvature;
		double ht = frontLane->LaneNode[k].heading;
		//DEBUG("===zLM::DealWithNode===frontLan->node_Heading = %f\r\n", ht);


		//获取地图点附近的坡度信息，Ramp，lkw，20201207
		double rs = frontLane->LaneNode[k-1].slopev;
		double rt = frontLane->LaneNode[k].slopev;
		double sengmentDis = 0.0;
		uint32 Loc_nodenum = 0;
		double Loc_distance = 0;
		if (k == 1)
		{
			Loc_distance = 0;
		}
		else
		{
			Loc_nodenum = outglobalTraj.nodeNum;
			//DEBUG("===DealWithNode===outglobalTraj.nodeNum = %d\r\n", Loc_nodenum);
			Loc_distance = outglobalTraj.laneNodeInfos[Loc_nodenum - 1].s;
		}
		//对曲率和横纵坐标进行插值
		int ret_line = generateTrajWithLineCurvature(Loc_distance, xs, ys, cs, hs,rs,xt, yt, ct, ht,rt,&outglobalTraj, &sengmentDis);
		//outglobalTraj.laneNodeInfos[k].s += sengmentDis;
		//先将坐标点进行插值

		if (outglobalTraj.nodeNum >= MAX_TRAJ_POINT_NUM)
		{
			break;
		}
	}
	laneNode localLaneNodes;

	memset(&localLaneNodes, 0, sizeof(laneNode));
	//DEBUG("===DealWithNode===localLaneNodes.Curvature = %f\r\n",localLaneNodes.curvature);
	//将插值后的坐标进行坐标系转换
	for (uint32 m = 0; m < outglobalTraj.nodeNum; m++)
	{
		double gx = outglobalTraj.laneNodeInfos[m].x;
		double gy = outglobalTraj.laneNodeInfos[m].y;

		// std::cout<<"TEST: gx gy :" << gx << " " <<gy << std::endl;
		double sengmentDis = 0.0;
		int ret_trans = decision::getGridCoordiFromParkXY(heading, mapHeading, xVeh, yVeh, gx, gy, &localLaneNodes.x, &localLaneNodes.y);
		if (m == 0)
		{
			if (localLaneNodes.x > 0)
			{
				sengmentDis = decision::Distance(0, 0, localLaneNodes.x, localLaneNodes.y);
				localLaneNodes.s = dis + sengmentDis;
			}
			else
			{
				localLaneNodes.s = 0;
			}
			sengmentDis = dis + sengmentDis;
		}
		else
		{
			double last_gx = outglobalTraj.laneNodeInfos[m - 1].x;
			double last_gy = outglobalTraj.laneNodeInfos[m - 1].y;
			sengmentDis = decision::Distance(last_gx, last_gy, gx, gy);
			//localLaneNodes.s = outglobalTraj.laneNodeInfos[m - 1].s + sengmentDis;
		}
		localLaneNodes.s = outglobalTraj.laneNodeInfos[m].s + dis;
		localLaneNodes.curvature=outglobalTraj.laneNodeInfos[m].curvature;
		localLaneNodes.heading = outglobalTraj.laneNodeInfos[m].heading;
			// DEBUG("===DealWithNode===localLaneNodes.Curvature = %f\r\n",localLaneNodes.curvature);
		if (localLaneNodes.x > 0)
		{
			//车正前方的点进行保留
			globalTraj->laneNodeInfos.push_back(localLaneNodes);
			nodeNum += 1;
		}
	}
	globalTraj->nodeNum = nodeNum;
	/*对POI点进行坐标转换*/
}

/*******************************************************
*input: 起点、终点，包括横纵坐标、曲率和坡度
*
*output:填充后的轨迹，填充后轨迹的长度，这个是对xg yg操作的
*
*funtction:在两点之间填充直线轨迹
*
*note:是在栅格图坐标系下的，即车后方x正方向，车右方y轴正方向
*******************************************************/
int decision::generateTrajWithLineCurvature(double distance, double xs, double ys, double cs, double hs, double rs,double xt, double yt, double ct, double ht,double rt, laneInfo *frontTraj, double *sengmentDis)
{
	int ret = FAIL;
	//计算两点之间距离，栅格图坐标系下，xs一定比xt大
	double deltaX = xt - xs;
	double deltaY = yt - ys;
	double deltaC = ct - cs;
	double deltaH = ht - hs;
	double dis = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
	double deltaR = rt - rs;
	if (dis <= NODE_SIZE)
	{
		//不能超过数组定义长度上限
		if (frontTraj->nodeNum > MAX_TRAJ_POINT_NUM)
		{
			return FAIL;
		}
		laneNode localLaneNodes;
		memset(&localLaneNodes, 0, sizeof(laneNode));
		localLaneNodes.x = xs;
		localLaneNodes.y = ys;
		localLaneNodes.curvature = cs;
		localLaneNodes.heading = hs;
		localLaneNodes.s = dis;
		localLaneNodes.slopev = rs;

		frontTraj->laneNodeInfos.push_back(localLaneNodes);
		frontTraj->nodeNum += 1;
		return SUCCEED; //如果不足10cm，则不再插值
	}
	//计算两关键点之间应该有几个间隔
	int NodeNum = dis / NODE_SIZE;
	*sengmentDis = 0.0;
	//计算x y方向上节点递增距离
	if (NodeNum > 0)
	{
		//
		double signleDeltaX = (deltaX / NodeNum);
		double signleDeltaY = (deltaY / NodeNum);
		double signleDeltaC = (deltaC / NodeNum);
		double signleDeltaH = (deltaH / NodeNum);
		double singleDeltaR = (deltaR / NodeNum);
		//ROS_INFO("deltaX is %.3f, NodeNum is %d, singleDeltaX is %.3f", deltaX, NodeNum, signleDeltaX);
		//sdj  chenge 202020610
		laneNode localLaneNodes;
		memset(&localLaneNodes, 0, sizeof(laneNode));
		for (int j = 0; j < NodeNum; j++)
		{
			//不能超过数组定义长度上限

			localLaneNodes.x = xs + j * signleDeltaX;
			localLaneNodes.y = ys + j * signleDeltaY;
			localLaneNodes.curvature = cs + j * signleDeltaC;
			//localLaneNodes.s = 0;
			localLaneNodes.s = *sengmentDis + distance;
			localLaneNodes.heading = hs + j * signleDeltaH;
			localLaneNodes.slopev = rs + j * singleDeltaR;
			frontTraj->laneNodeInfos.push_back(localLaneNodes);
			//frontTraj->laneNodeInfos.push_back(localLaneNodes);
			//printf("=888=56=decision=localLaneNodes.curvature  %f \r\n",localLaneNodes.curvature);
			if (j >= 1)
			{
				double step_distance2 = 0.0;
				step_distance2 = decision::Distance(frontTraj->laneNodeInfos[frontTraj->nodeNum - 1].x,frontTraj->laneNodeInfos[frontTraj->nodeNum - 1].y,frontTraj->laneNodeInfos[frontTraj->nodeNum].x,frontTraj->laneNodeInfos[frontTraj->nodeNum].y);
				*sengmentDis += step_distance2;
				//ROS_INFO("step_distance2 is %.8f, *sengmentDis is %.8f", step_distance2, *sengmentDis);
			}
			frontTraj->nodeNum += 1;
			if (frontTraj->nodeNum >= MAX_TRAJ_POINT_NUM)
			{
				break;
			}
			//frontTraj->laneNodeInfos.push_back(localLaneNodes);
		}
	}
	ret = SUCCEED;
	return ret;
}


/**************************************************************************************************
Function:           
Description:    把全局轨迹从园区坐标系转到车辆坐标系下
Calls:              
Called By:          
Table Accessed:     
Table Updated:      
Input:      车头航向、园区坐标系y轴与北向夹角、车辆在园区坐标系下x、y坐标，轨迹点在园区坐标系下x、y坐标    
Output:     轨迹点在车辆坐标系下x、y坐标      
Return:             
Others:             
**************************************************************************************************/
int decision::getGridCoordiFromParkXY(const double veh_heading, const double park_y_heading, const double x_veh, 
	const double y_veh, const double x_pt, const double y_pt, double* x_out, double* y_out)
{
	//当前从定位获取的车辆航向是与正北方向逆时针夹角，所以正北方向到车辆坐标系x轴逆时针夹角为：
	double veh_x_heading = veh_heading;
	//处理到0-360度范围内
	if (veh_x_heading >= 360.0){
		veh_x_heading = veh_x_heading - 360.0;
	}
	else if (veh_x_heading < 0.0){
		veh_x_heading = veh_x_heading + 360.0;
	}
	else {}
	//园区坐标系y轴与正北方向逆时针夹角是park_y_heading，逆时针为正，所以正北到园区坐标系x轴逆时针夹角为：
	double park_x_heading = 360.0 - (90.0 - park_y_heading);
	if (park_x_heading >= 360.0){
		park_x_heading = park_x_heading - 360.0;
	}
	else if (park_x_heading < 0.0){
		park_x_heading = park_x_heading + 360.0;
	}
	else {}
	//车辆坐标系x轴航向减园区坐标系x轴航向即为二者夹角
	double delta_heading = veh_x_heading - park_x_heading;
	if (delta_heading >= 360.0){
		delta_heading = delta_heading - 360.0;
	}
	else if (delta_heading < 0.0){
		delta_heading = delta_heading + 360.0;
	}
	else {}
	//转为弧度
	delta_heading = delta_heading * M_PI / 180.0;

	//用坐标系偏移公式求出车辆坐标系下的坐标
	*x_out = (y_pt - y_veh)*sin(delta_heading) + (x_pt - x_veh)*cos(delta_heading);
	*y_out = (y_pt - y_veh)*cos(delta_heading) - (x_pt - x_veh)*sin(delta_heading);

	return SUCCEED;
}


double decision::Distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int decision::ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = decision::Distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}
	return closestWaypoint;
}

int decision::NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
{

	int closestWaypoint = decision::ClosestWaypoint(x,y,maps_x,maps_y);
	// std::cout <<"zlm::NextWaypoint: closestWaypoint=  " << closestWaypoint <<std::endl;
	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = fabs(theta-heading);

	if(angle > M_PI/4)
	{
		closestWaypoint++;
	}
	// std::cout <<"zlm::NextWaypoint: NextWaypoint=  " << closestWaypoint <<std::endl;
	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> decision::GetFrenet(double x, double y, double theta, const std::vector<double> &maps_x, \
    const std::vector<double> &maps_y)
{
	std::vector<double> res_;
	int next_wp = decision::NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;

	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = decision::Distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = decision::Distance(center_x,center_y,x_x,x_y);
	double centerToRef = decision::Distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}
	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += decision::Distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += decision::Distance(0,0,proj_x,proj_y);
	res_.push_back(frenet_s);
	res_.push_back(frenet_d);
	// std::cout << "frenet_s: " << frenet_s << "  frenet_d: " <<frenet_d <<std::endl;
	return res_;

}
/**
 * @brief generate a refpath which help to make a further procession in frenet coordinate
 * update:
 * 		V2.1 add logic: using pathLane as refline during Decision Command LEFTAVOID RIGHT AVOID 
 * @param onpath   in current time, if ADC in ego lane
 * @param ..
 * @return refpath, 
*/
void decision::GetRefpath(const int &onpath, hdMapTrajectory *Trajectory, laneInfo *refpath, Dt_RECORD_LocalizationInfo *localizationInfo, decision_info *decisionInfo)
{
	hdmapPathInfos path_;
	// when vehicle onpath, or duiring LEFT AVOID Fun, set global path as refpath. else set local path
	if(onpath==true || decisionInfo->decision_command == LEFTAVOID || decisionInfo->decision_command == RIGHTAVOID)
	{
		path_ = Trajectory->pathLane[0];
		DEBUG("zlm::get_refpath: path_ = Trajectory->pathLane[0] \r\n"); // zlm 2021-01-23 add
	}
	else
	{
		path_ = Trajectory->localPath[0];
		DEBUG("zlm::get_refpath: path_ = Trajectory->localPath \r\n"); // zlm 2021-01-23 add
	}
	
	for (uint32 lane_seg_idx = 0; lane_seg_idx < path_.segNum; lane_seg_idx++)// range all lane segments
	{
		for(uint32 path_node_idx = 0; path_node_idx < path_.hdmapPathInfo[lane_seg_idx].laneInfos[0].nodeNum; path_node_idx+=5) 
		{
			
			laneNode node_; // add temp node
			memset(&node_, 0, sizeof(laneNode));
			node_.x =  path_.hdmapPathInfo[lane_seg_idx].laneInfos[0].laneNodeInfos[path_node_idx].x;
			node_.y =  path_.hdmapPathInfo[lane_seg_idx].laneInfos[0].laneNodeInfos[path_node_idx].y;
			node_.heading = path_.hdmapPathInfo[lane_seg_idx].laneInfos[0].laneNodeInfos[path_node_idx].heading - localizationInfo->yaw;
			refpath->laneNodeInfos.push_back(node_);
		}
	}
	
	refpath->nodeNum = refpath->laneNodeInfos.size(); 
	DEBUG("zlm::get_refpath: refpath->nodeNum  = %ld \r\n",refpath->laneNodeInfos.size());
	DEBUG("zlm::get_refpath: node_idx as follow \r\n");
	for(uint32 node_idx =0; node_idx < refpath->laneNodeInfos.size();node_idx+=2)
	{
		DEBUG("zlm::get_refpath: [node_idx_x, node_idx_y, node_idx_heading] = %f %f %f\r\n" ,refpath->laneNodeInfos[node_idx].x, \
			refpath->laneNodeInfos[node_idx].y ,refpath->laneNodeInfos[node_idx].heading);
	}
	
}

/**
 *  @brief get the segments' boundary of ego lane.
		  this fun try to solve each sgement's nearest distance and farest distance and it's lane width;
    @param ..
	@param   segs_boundary, the boundary of each segment, include remote distance and near distance and lanewidth.
	@update: V2.0 fix bug , which there is zero node in some segment make errors
*/
void decision::GetSegsBoundary(int onpath, hdMapTrajectory* Trajectory, Dt_RECORD_HdmapFrontPLane* globePLane, \
	Dt_RECORD_HdmapLocalLane* localPLanne, Dt_RECORD_LocalizationInfo *localizationInfo, const std::vector<double> &refpath_x, const std::vector<double> &refpath_y, \
	std::vector<std::vector<double>> &segs_boundary)
{
	hdmapPathInfos path_; // temp value, which help th sync trajectory 
	double seg_lane_width;
	static double valid_laneWidth =2.8;
	if (onpath)
		path_ = Trajectory->pathLane[0];
	else
		path_ = Trajectory->localPath[0];
	DEBUG("OBJ SELECTION----> segments nums is %d\r\n",path_.segNum);
	for (uint32 seg_idx = 0; seg_idx < path_.segNum; ++seg_idx)
	{
		// get key point of each segment
		double node_num = path_.hdmapPathInfo[seg_idx].laneInfos[0].nodeNum;
		if (node_num < 2) 
		{
			DEBUG("OBJ SELECTION----> ERROR : There is only %f points in segments[%d]\r\n",node_num, seg_idx);
			continue;
		}
		double x_s = path_.hdmapPathInfo[seg_idx].laneInfos[0].laneNodeInfos[0].x;
		double y_s = path_.hdmapPathInfo[seg_idx].laneInfos[0].laneNodeInfos[0].y;
		double h_s = path_.hdmapPathInfo[seg_idx].laneInfos[0].laneNodeInfos[0].heading - localizationInfo->yaw;
		double x_e = path_.hdmapPathInfo[seg_idx].laneInfos[0].laneNodeInfos[node_num - 1].x;
		double y_e = path_.hdmapPathInfo[seg_idx].laneInfos[0].laneNodeInfos[node_num - 1].y;
		double h_e = path_.hdmapPathInfo[seg_idx].laneInfos[0].laneNodeInfos[node_num - 1].heading - localizationInfo->yaw;

		// get each sgement nearest distance Seg_S_s / farest disatance Seg_S_e and lane width
		double seg_S_s = decision::GetFrenet(x_s, y_s, h_s, refpath_x, refpath_y)[0];
		double seg_S_e = decision::GetFrenet(x_e, y_e, h_e, refpath_x, refpath_y)[0];
		if (onpath)
			seg_lane_width = globePLane->PlanSeg[seg_idx].Lane[0].lane_width;
		else // not on path
		{
			if (seg_idx == 0)
				seg_lane_width = localPLanne->LocalLane->lane_width;
			else
				seg_lane_width = localPLanne->NextSeg[seg_idx - 1].Lane[0].lane_width;
		}
		if(seg_lane_width != 0) // save valid lane width
			valid_laneWidth = seg_lane_width;
		else // when meet crossing, pad last valid lane width
			seg_lane_width = valid_laneWidth;

		std::vector<double> seg_boudary;
		if(seg_idx == 0) // first segment's should count from vhehicle loclization
			seg_S_s =0;
		seg_boudary.push_back(seg_S_s);
		seg_boudary.push_back(seg_S_e);
		seg_boudary.push_back(seg_lane_width);
		segs_boundary.push_back(seg_boudary);
	}
	DEBUG("OBJ SELECTION----> segments bodunary:\r\n");
	for (uint32 seg_idx = 0; seg_idx < segs_boundary.size(); ++seg_idx)
	{
		DEBUG("OBJ SELECTION----> segs_idx, start_s, end_s, lanewidth = %d  %2f  %2f  %2f\r\n", seg_idx,\
			segs_boundary[seg_idx][0], segs_boundary[seg_idx][1], segs_boundary[seg_idx][2]);
	}
}
/**
 * @brief ObjDectect fun, help to select interested objs in diff lane with " Obj List"
 * updae: V4.3 2020-02-01 update Midside sel range 
 *		  V4.2 2020-01-22 close side detect; comment format longtitude frenet s with ego vehl;
 *		  V4.1 2020-01-18 add Left and Right Side detect logic
 *        V4.0 2020-01-07 
 * @param onpath 
 * @param Trajectory
 * @param ...
 * @return selectObj, selected objs 
 * 
 * */
void decision::ObjDetect(int onpath, hdMapTrajectory *Trajectory, Dt_RECORD_HdmapInfo *hdmapInfos,\
     Dt_RECORD_HdmapFrontPLane *globePLane, Dt_RECORD_HdmapLocalLane *localPLanne,  \
     Dt_RECORD_LocalizationInfo *localInfos, Dt_RECORD_EnvModelInfos *envModelInfo, EgoConfigPara ego_config, decision_info *decisionInfo, objSecList *selectObj)
{
	// initialization 
	memset(selectObj,0,sizeof(objSecList)); // clear output

	double max_valid_s =30;
	std::vector<uint32> obstalce_cipv_flag(5); // 0->mid  1->midLeft 2->midRight 3->left 4->right
	std::vector<double> obstalce_cipv_s(5, max_valid_s); 
	std::vector<double> obstalce_cipv_d(5); 
	std::vector<uint32> obstalce_cipv_idx(5); 


	double vehicle_width = 1.93;
	double pedestain_width = 0.5;// zlm 2021-0121 change from 1 to 0.5
	double safeDis = 0.2; 
	double laneWidth = 0;
	double safeWidth =  vehicle_width + safeDis * 2;
    static double last_laneWidth = safeWidth ; // last not zero lane width, default as safe lanewidth,
											  // deal with scenario which start at crossing 

	if(envModelInfo->Lanes.LaneLines->confidence > 80)  // ENV  laneInfo  (0-100)
	{
		laneWidth = envModelInfo->Lanes.width;
		DEBUG("OBJ SELECTION----> lane_confidence, lane_width = %d %f \r\n", envModelInfo->Lanes.LaneLines->confidence, laneWidth);
	}
		
	else	// HDMAP
	{
		if(onpath)   
			laneWidth = globePLane->PlanSeg[0].Lane[0].lane_width;
		else
			laneWidth = localPLanne->LocalLane[0].lane_width;

	}
	if(laneWidth !=0) // save valid lane width(not zero) 
		last_laneWidth = laneWidth;
	else //crossing, read from last valid width
		laneWidth = last_laneWidth;

	laneInfo refpath; //define refpath
	memset(&refpath, 0, sizeof(laneInfo));
	decision::GetRefpath(onpath, Trajectory, &refpath, localInfos, decisionInfo);
	// decision::convert_flat_to_vehicle(localInfos, &refpath);  // get refpath in vehicle coordinate
	if(refpath.nodeNum ==0) 
	{
		DEBUG("zlm::obj_sel: no refpath\r\n");
		DEBUG("OBJ SELECTION----> onpath= %d\r\n", onpath);
		DEBUG("OBJ SELECTION----> selectObj->frontMid.postion          = %d \r\n", selectObj->frontMid.postion);
		DEBUG("OBJ SELECTION----> selectObj->frontMid.obj.id           = %d \r\n", selectObj->frontMid.obj.id);
		DEBUG("OBJ SELECTION----> selectObj->frontMid.obj.s            = %f \r\n", selectObj->frontMid.obj.s);
		DEBUG("OBJ SELECTION----> selectObj->frontMid.obj.d            = %f \r\n", selectObj->frontMid.obj.d);
		DEBUG("OBJ SELECTION----> selectObj->frontMid.obj.pos_x        = %f \r\n", selectObj->frontMid.obj.pos_x);
		DEBUG("OBJ SELECTION----> selectObj->frontMid.obj.pos_y        = %f \r\n", selectObj->frontMid.obj.pos_y);
		DEBUG("OBJ SELECTION----> selectObj->frontMid.obj.heading      = %f \r\n", selectObj->frontMid.obj.heading);
		DEBUG("OBJ SELECTION----> selectObj->frontMid.obj.rel_speed_x  = %f \r\n", selectObj->frontMid.obj.rel_speed_x);
		DEBUG("OBJ SELECTION----> selectObj->frontMid.obj.rel_speed_y  = %f \r\n", selectObj->frontMid.obj.rel_speed_y);
		DEBUG("OBJ SELECTION----> selectObj->frontMid.obj.abs_speed_x  = %f \r\n", selectObj->frontMid.obj.abs_speed_x);
		DEBUG("OBJ SELECTION----> selectObj->frontMid.obj.type         = %d \r\n", selectObj->frontMid.obj.type);
		return;
	}
	// get refpath_x, refpath_y
	std::vector<double> refpath_x;
	std::vector<double> refpath_y;
	for(uint32 point_idx = 0; point_idx < refpath.nodeNum; ++point_idx)
	{
		refpath_x.push_back(refpath.laneNodeInfos[point_idx].x);
		refpath_y.push_back(refpath.laneNodeInfos[point_idx].y);
	}
	
	// calculate segments boundary
	std::vector<std::vector<double>> segs_boundary;
	double segs_max_width = laneWidth;

	// calculate max lanewidth to process pre-select, and get each segment's boundary
	GetSegsBoundary(onpath, Trajectory, globePLane, localPLanne, localInfos, refpath_x, refpath_y, segs_boundary);
	for (uint32 seg_idx = 0; seg_idx < segs_boundary.size(); ++seg_idx)
	{
		if (segs_boundary[seg_idx].back() > segs_max_width)
			segs_max_width = segs_boundary[seg_idx].back(); // iterate each seg's width
		
		DEBUG("OBJ SELECTION----> segs_boundary[%d].laneNumber = %d \r\n", seg_idx, globePLane->PlanSeg[seg_idx].lane_count);

		if(segs_boundary[seg_idx].back() ==0) // fix slove error
		{
			DEBUG("OBJ SELECTION----> ERROR: segs_boundary[%d].back() = %f, and now padded by %f\r\n",seg_idx, segs_max_width, laneWidth);
			segs_boundary[seg_idx].back() = laneWidth;
		}
	}
	DEBUG("OBJ SELECTION----> segs_max_width = %2f\r\n", segs_max_width);

	// calculate ego vehicle pos's s and d
	std::vector<double> loc_sd = decision::GetFrenet(0,0,0, refpath_x, refpath_y);
			
	std::cout << "OBJ SELECTION----> localization_sd :" << loc_sd[0] <<" " << loc_sd[1] <<std::endl;
	
	DEBUG("OBJ SELECTION----> envModelInfo->obstacle_num= %d\r\n", envModelInfo->obstacle_num);
	DEBUG("OBJ SELECTION----> current lane_width / 2= %f\r\n", laneWidth / 2);

	//  iterater all obstacle to select CIPV
	for (uint32 obj_idx = 0; obj_idx < envModelInfo->obstacle_num; obj_idx++)
	{	
		if(envModelInfo->Obstacles[obj_idx].pos_x <0) // skip backward obj select
			continue;

		std::vector<double> obj_sd = decision::GetFrenet(envModelInfo->Obstacles[obj_idx].pos_x, envModelInfo->Obstacles[obj_idx].pos_y, \
			0, refpath_x, refpath_y); // not consider obs'heading
		
		// obj_sd[0] = obj_sd[0] - loc_sd[0]; // zlm 2021-122 commnent, not uncomment until solve back sel. update obstacle property s
		
		double obj_s = obj_sd[0];
		double obj_d = obj_sd[1]; 

		DEBUG("OBJ SELECTION----> obj_id, obj_s, obj_d, obj_posx, obj_posy:  %2d     %f     %f     %f    %f\r\n", envModelInfo->Obstacles[obj_idx].id, obj_s, obj_d, \
		envModelInfo->Obstacles[obj_idx].pos_x, envModelInfo->Obstacles[obj_idx].pos_y);
		

		// var 'vehilce_width/2'  help to consider obstacle's boundary( mid_point + vehicle_width/2 = side boundary)    
		// TO DO : should consider obstacle's  attitude(with obstacle's heading)

		// first check , check if ego vehicle will make collision with others during RANGE_1(safeWidth) -> (vehicle width + expand dis = 1.97 + 0.4)
		//
		//         Range1
		//     |    .<->.    |
		//     |    . _ .    |
		//     |    .| |.    |
		//	   |             |
		//     |    . _ .    |
		//     |    .| |.    |

		double obstacle_width = vehicle_width; // default obstacle as vehicle width	
		if(envModelInfo->Obstacles[obj_idx].type ==0) 
			obstacle_width = pedestain_width;

		double min_dis = 3.75; // distance from rear axis to vehicle head 
		if(decisionInfo->decision_command == LEFTAVOID || decisionInfo->decision_command == RIGHTAVOID)
			min_dis =0.1;

		// Mid Lane(mid)
		// (safeWidth + obstacle_width)/2  = |2.1| or |1.6|
		if(obj_d > -(safeWidth/2 + obstacle_width/2) && obj_d < (safeWidth/2 + obstacle_width/2) &&  obj_s > min_dis &&  obj_s < obstalce_cipv_s[0])
		{
			obstalce_cipv_flag[0] = 1; // CIPV_1 valid flag
			obstalce_cipv_s[0] = obj_s; // update min s
			obstalce_cipv_d[0] = obj_d; // update d
			obstalce_cipv_idx[0] = obj_idx;
		}

		// second check , check if obstacle appear in RANGE_2() -> (laneWidth)
		//
		//			Range2
		//     |<-->     <-->|
		//     |  _ .   .    |
		//     | | |.   .    |
		//     |    .   .    |
		//     |    . _ .    |
		//     |    .| |.    |
		
		
		// Mid Lane(midLeft && midRight)
		if (obj_d > -(segs_max_width/2 + obstacle_width/2) && obj_d < (segs_max_width/2 + obstacle_width/2) && obj_s > min_dis &&  obj_s < max_valid_s)
		{
			// select obstacle by itearte each segment boundary && Min distance S
			// segs_boundary, [0]->min_s  [1]->max_x [2]->width  of each segment
			for (uint32 seg_idx = 0; seg_idx < segs_boundary.size(); ++seg_idx)
			{
				// if (obj_s > segs_boundary[seg_idx][0] && obj_s < segs_boundary[seg_idx][1] && \
				// 	obj_d > -(segs_boundary[seg_idx][2]/2 + obstacle_width/2) && obj_d < -(safeWidth/2 + obstacle_width/2) &&\
				// 	obj_s < obstalce_cipv_s[1])
				// {
				// 	obstalce_cipv_flag[1] =1; //  (second cipv) valid flag 
				// 	obstalce_cipv_s[1] = obj_s; // update min s
				// 	obstalce_cipv_d[1] = obj_d; // update d
				// 	obstalce_cipv_idx[1] = obj_idx;
				// }

				// if (obj_s > segs_boundary[seg_idx][0] && obj_s < segs_boundary[seg_idx][1] && \
				// 	obj_d < (segs_boundary[seg_idx][2]/2 + obstacle_width/2) && obj_d > (safeWidth/2 + obstacle_width/2) && \
				// 	obj_s < obstalce_cipv_s[2])
				// {
				// 	obstalce_cipv_flag[2] =1; // (second cipv) valid flag 
				// 	obstalce_cipv_s[2] = obj_s; // update min s
				// 	obstalce_cipv_d[2] = obj_d; // update d
				// 	obstalce_cipv_idx[2] = obj_idx;
				// }


				//2021-0201 Update: As the basic point of objList’ ele will move  near ego car when it locate in side, so cancel obj expend logic will be more efficient
				// MidLeft
				if (obj_s > segs_boundary[seg_idx][0] && obj_s < segs_boundary[seg_idx][1] && \
					obj_d > -(segs_boundary[seg_idx][2]/2 ) && obj_d < -(safeWidth/2 + obstacle_width/2) &&\
					obj_s < obstalce_cipv_s[1])
				{
					obstalce_cipv_flag[1] =1; //  (second cipv) valid flag 
					obstalce_cipv_s[1] = obj_s; // update min s
					obstalce_cipv_d[1] = obj_d; // update d
					obstalce_cipv_idx[1] = obj_idx;
				}
				// MidRight
				if (obj_s > segs_boundary[seg_idx][0] && obj_s < segs_boundary[seg_idx][1] && \
					obj_d < (segs_boundary[seg_idx][2]/2 ) && obj_d > (safeWidth/2 + obstacle_width/2 ) && \
					obj_s < obstalce_cipv_s[2])
				{
					obstalce_cipv_flag[2] =1; // (second cipv) valid flag 
					obstalce_cipv_s[2] = obj_s; // update min s
					obstalce_cipv_d[2] = obj_d; // update d
					obstalce_cipv_idx[2] = obj_idx;
				}
			}
		}

		// Side Lane
		if (obj_d > -2 * segs_max_width  && obj_d < 2 * segs_max_width  &&  obj_s < max_valid_s)
		{
			// select obstacle by itearte each segment boundary && Min distance S
			// segs_boundary, [0]->min_s  [1]->max_x [2]->width  of each segment
			for (uint32 seg_idx = 0; seg_idx < segs_boundary.size(); ++seg_idx)
			{
				// Left Lane
				if (obj_s > segs_boundary[seg_idx][0] && obj_s < segs_boundary[seg_idx][1] && \
					obj_d > -(segs_boundary[seg_idx][2] + safeWidth/2) && obj_d < -(segs_boundary[seg_idx][2] - safeWidth/2) &&\
					obj_s < obstalce_cipv_s[3])
				{
					obstalce_cipv_flag[3] =1; // (left Lane) valid flag 
					obstalce_cipv_s[3] = obj_s; // update min s
					obstalce_cipv_d[3] = obj_d; // update d
					obstalce_cipv_idx[3] = obj_idx;
				}

				// Right Lane
				if (obj_s > segs_boundary[seg_idx][0] && obj_s < segs_boundary[seg_idx][1] && \
					obj_d < (segs_boundary[seg_idx][2] + safeWidth/2) && obj_d > (segs_boundary[seg_idx][2] - safeWidth/2) && \
					obj_s < obstalce_cipv_s[4])
				{
					obstalce_cipv_flag[4] =1; //  (right lane) valid flag 
					obstalce_cipv_s[4] = obj_s; // update min s
					obstalce_cipv_d[4] = obj_d; // update d
					obstalce_cipv_idx[4] = obj_idx;
				}
				// 2021- 0121  help to deal with side lane miss dectect
				double lanenum = 4; // have both left and right lane
				if(onpath)
					lanenum = Trajectory->pathLane[0].hdmapPathInfo[0].laneNum;
				else
					lanenum = Trajectory->localPath[0].hdmapPathInfo[0].laneNum;

				
				lanenum =1; //zlm TEMP  !!!! 2021- 01-22 close side lane detect
				if(lanenum == 2 || lanenum ==1)// no right lane  0 crossing , 1 only ego, 2 only left,  3 only right, 4 have both
				{
					obstalce_cipv_flag[4] =0; //  (right lane) valid flag 
					obstalce_cipv_s[4] = 0; // update min s
					obstalce_cipv_d[4] = 0; // update d
					obstalce_cipv_idx[4] = 0;	
				}
				if(lanenum==3 || lanenum ==1) // no left lane
				{
					obstalce_cipv_flag[3] =0; //  (left lane) valid flag 
					obstalce_cipv_s[3] = 0; // update min s
					obstalce_cipv_d[3] = 0; // update d
					obstalce_cipv_idx[3] = 0;	
				}
			}
		}
	}

	// Assign property
	// 2021-01-07 zlm , check mid left and mid right no matter if mid exist obstacle 
	if(obstalce_cipv_flag[0]== 1) // CIPV_Mid
	{
		selectObj->frontMid.postion = 1;
		selectObj->frontMid.obj.s = obstalce_cipv_s[0];
		selectObj->frontMid.obj.d = obstalce_cipv_d[0];
		// obstalce_cipv_idx = obstalce_cipv_idx[0];
		AssignObstacleProperty(selectObj->frontMid.obj, envModelInfo->Obstacles[obstalce_cipv_idx[0]]); // assign other property 
	}

	if( obstalce_cipv_flag[1] ==1) // CIPV_Midleft
	{
		selectObj->frontMidLeft.postion = 1;	
		selectObj->frontMidLeft.obj.s = obstalce_cipv_s[1];
		selectObj->frontMidLeft.obj.d = obstalce_cipv_d[1];
		// obstalce_cipv_idx = obstalce_cipv_V2_idx;	
		AssignObstacleProperty(selectObj->frontMidLeft.obj, envModelInfo->Obstacles[obstalce_cipv_idx[1]]);
	}

	if(obstalce_cipv_flag[2] ==1) // CIPV_MidRight
	{
		selectObj->frontMidRight.postion = 1;	
		selectObj->frontMidRight.obj.s = obstalce_cipv_s[2];
		selectObj->frontMidRight.obj.d = obstalce_cipv_d[2];
		// obstalce_cipv_idx = obstalce_cipv_V3_idx;	
		AssignObstacleProperty(selectObj->frontMidRight.obj, envModelInfo->Obstacles[obstalce_cipv_idx[2]]);
	}
	
	if(obstalce_cipv_flag[3] ==1) // CIPV_Left
	{
		selectObj->frontLeft.postion = 1;	
		selectObj->frontLeft.obj.s = obstalce_cipv_s[3];
		selectObj->frontLeft.obj.d = obstalce_cipv_d[3];
		// obstalce_cipv_idx = obstalce_cipv_V3_idx;	
		AssignObstacleProperty(selectObj->frontLeft.obj, envModelInfo->Obstacles[obstalce_cipv_idx[3]]);
	}

	if(obstalce_cipv_flag[4] ==1) // CIPV_Right
	{
		selectObj->frontRight.postion = 1;	
		selectObj->frontRight.obj.s = obstalce_cipv_s[4];
		selectObj->frontRight.obj.d = obstalce_cipv_d[4];
		// obstalce_cipv_idx = obstalce_cipv_V3_idx;	
		AssignObstacleProperty(selectObj->frontRight.obj, envModelInfo->Obstacles[obstalce_cipv_idx[4]]);
	}
	// Print property

	// //if(selectObj->frontMid.postion == 1) // ONLY print with success selection 
	DebugObstacleProperty("frontMid",       selectObj->frontMid);
	DebugObstacleProperty("frontMidLeft",   selectObj->frontMidLeft);
	DebugObstacleProperty("frontMidRight",  selectObj->frontMidRight);
	DebugObstacleProperty("frontLeft",      selectObj->frontLeft);
	DebugObstacleProperty("frontRight",     selectObj->frontRight);
}

/**
* @brief help to assign env obstacle to selected obstacle property(assign right to left)
* @param left, object of selected obstalce
* @param right,  object of to be assigned
*/
void decision::AssignObstacleProperty(Obj_sel &left, const Dt_RECORD_Obstacles &right)
{
	left.id          = right.id;
	left.type        = right.type;
	left.pos_x       = right.pos_x;
	left.pos_y       = right.pos_y;
	left.heading     = right.heading;
	left.rel_speed_x = right.rel_speed_x;
	left.rel_speed_y = right.rel_speed_y;
	left.abs_speed_x = right.abs_speed_x;
	left.abs_speed_y = right.abs_speed_y;
}

/**
* @brief help to assign env obstacle to selected obstacle property(assign right to left)
* @param str, to distinguish diff lane of part within one lane 
* @param obstacle, obstacle's property
*/
void decision::DebugObstacleProperty(const string &str, const objSec &obstacle)
{
	DEBUG("OBJ SELECTION----> selectObj->%s.postion          =  %d \r\n", str.c_str(), obstacle.postion);
	DEBUG("OBJ SELECTION----> selectObj->%s.obj.id           =  %d \r\n", str.c_str(), obstacle.obj.id);
	DEBUG("OBJ SELECTION----> selectObj->%s.obj.s            =  %f \r\n", str.c_str(), obstacle.obj.s);
	DEBUG("OBJ SELECTION----> selectObj->%s.obj.d            =  %f \r\n", str.c_str(), obstacle.obj.d);
	DEBUG("OBJ SELECTION----> selectObj->%s.obj.pos_x        =  %f \r\n", str.c_str(), obstacle.obj.pos_x);
	DEBUG("OBJ SELECTION----> selectObj->%s.obj.pos_y        =  %f \r\n", str.c_str(), obstacle.obj.pos_y);
	DEBUG("OBJ SELECTION----> selectObj->%s.obj.heading      =  %f \r\n", str.c_str(), obstacle.obj.heading);
	DEBUG("OBJ SELECTION----> selectObj->%s.obj.rel_speed_x  =  %f \r\n", str.c_str(), obstacle.obj.rel_speed_x);
	DEBUG("OBJ SELECTION----> selectObj->%s.obj.rel_speed_y  =  %f \r\n", str.c_str(), obstacle.obj.rel_speed_y);
	DEBUG("OBJ SELECTION----> selectObj->%s.obj.abs_speed_x  =  %f \r\n", str.c_str(), obstacle.obj.abs_speed_x);
	DEBUG("OBJ SELECTION----> selectObj->%s.obj.type         =  %d \r\n", str.c_str(), obstacle.obj.type);
}