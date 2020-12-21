#include "../libs/obstacle_selection.h"

double decision::distance(double x1, double y1, double x2, double y2)
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
		double dist = decision::distance(x,y,map_x,map_y);
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
std::vector<double> decision::getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, \
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

	double frenet_d = decision::distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = decision::distance(center_x,center_y,x_x,x_y);
	double centerToRef = decision::distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}
	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += decision::distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += decision::distance(0,0,proj_x,proj_y);
	res_.push_back(frenet_s);
	res_.push_back(frenet_d);

	return res_;

}

///***************************************************************************************************
//* 功  能:  坐标系转换: 地图坐标系 -> 车体坐标系
//* 处  理： 车辆当前位置作为原点, 将地图坐标系的点进行x,y的距离偏移,同时完成角度变换;
//* 输  入:       车辆定位信息(地图坐标系)
//				  参考路径信息(地图坐标系)
//*	
//		
//* 输  出: 参考路径信息(车体坐标系)

//***************************************************************************************************/
void decision::convert_flat_to_vehicle(Dt_RECORD_LocalizationInfo *loc_info, laneInfo *refpath){
     double a = loc_info->LocalizationResult.x;
    double b = loc_info->LocalizationResult.y;
    double t = -loc_info->yaw * M_PI/180; // base direction not sure
	// std::cout <<"zlm::convert_flat_to_vehicle: node_idx as follow /" << std::endl;
	// std::cout <<"zlm::convert_flat_to_vehicle: loc_info->LocalizationResult.x : " << a << std::endl;
	// std::cout <<"zlm::convert_flat_to_vehicle: loc_info->LocalizationResult.y: " << b << std::endl;
	// std::cout <<"zlm::convert_flat_to_vehicle: -loc_info->yaw * M_PI/180 : " << t << std::endl;

    for(uint32 i=0;i<refpath->nodeNum;++i)
    {
        double x_ = ( refpath->laneNodeInfos[i].x - a)*cos(t)   + (refpath->laneNodeInfos[i].y - b)*sin(t);
        double y_ = ( refpath->laneNodeInfos[i].x - a)*-sin(t)  + (refpath->laneNodeInfos[i].y - b)*cos(t);
		refpath->laneNodeInfos[i].x = x_;
		refpath->laneNodeInfos[i].y = y_;
    }

	// DEBUG("zlm::convert_flat_to_vehicle: refpath->nodeNum  = %d \r\n",refpath->laneNodeInfos.size());
	// for(uint32 node_idx =0; node_idx < refpath->laneNodeInfos.size();++node_idx)
	// {
	// 	DEBUG("zlm::convert_flat_to_vehicle: node_idx as follow \r\n/");
	// 	DEBUG("zlm::convert_flat_to_vehicle: node_idx_x %f: \r\n", refpath->laneNodeInfos[node_idx].x );
	// 	DEBUG("zlm::convert_flat_to_vehicle: node_idx_y : %f\r\n", refpath->laneNodeInfos[node_idx].y );
	// 	DEBUG("zlm::convert_flat_to_vehicle: node_idx_heading %f: \r\n", refpath->laneNodeInfos[node_idx].heading);
	// }
}

///***************************************************************************************************
//* 功  能:  生成参考路径refpath
//* 处  理： 根据onpath条件,输出参考路径refpath
//* 输  入:   onpath  自车是否在路径上
//           hdMapTrajectory Trajectory 前方路径	
//* 输  出:   laneInfo refpath 前方路径信息
//***************************************************************************************************/
void decision::get_refpath(const int &onpath, hdMapTrajectory *Trajectory, laneInfo *refpath){
		if(onpath==true)
	{
		for (uint32 lane_seg_idx = 0; lane_seg_idx < Trajectory->pathLane[0].segNum; lane_seg_idx++)// range all lane segments
		{
			for(uint32 path_node_idx = 0; path_node_idx < Trajectory->pathLane[0].hdmapPathInfo[lane_seg_idx].laneInfos[0].nodeNum; path_node_idx+=5)
			{
				
				laneNode node_; // add temp node
				memset(&node_, 0, sizeof(laneNode));
				node_.x =  Trajectory->pathLane[0].hdmapPathInfo[lane_seg_idx].laneInfos[0].laneNodeInfos[path_node_idx].x;
				node_.y=  Trajectory->pathLane[0].hdmapPathInfo[lane_seg_idx].laneInfos[0].laneNodeInfos[path_node_idx].y;
				node_.heading =  Trajectory->pathLane[0].hdmapPathInfo[lane_seg_idx].laneInfos[0].laneNodeInfos[path_node_idx].heading;
				refpath->laneNodeInfos.push_back(node_);
				// DEBUG("zlm::get_refpath: Trajectory_x : %f\r\n" ,node_.x);
				// DEBUG("zlm::get_refpath: Trajectory_y : %f\r\n", node_.y );
				// DEBUG("zlm::get_refpath: Trajectory_heading : %f\r\n " , node_.heading);
			}
		}
	}
	else // onpath == false
	{
		for (uint32 lane_seg_idx = 0; lane_seg_idx < Trajectory->localPath[0].segNum; lane_seg_idx++)// range all lane segments
		{
			for(uint32 path_node_idx = 0; path_node_idx < Trajectory->localPath[0].hdmapPathInfo[lane_seg_idx].laneInfos[0].nodeNum; path_node_idx++)
			{
				laneNode node_; // add temp node
				memset(&node_, 0, sizeof(laneNode));
				node_.x =  Trajectory->localPath[0].hdmapPathInfo[lane_seg_idx].laneInfos[0].laneNodeInfos[path_node_idx].x;
				node_.y =  Trajectory->localPath[0].hdmapPathInfo[lane_seg_idx].laneInfos[0].laneNodeInfos[path_node_idx].y;
				node_.heading =  Trajectory->localPath[0].hdmapPathInfo[lane_seg_idx].laneInfos[0].laneNodeInfos[path_node_idx].heading;
				refpath->laneNodeInfos.push_back(node_);
			}
		}
		
	}
	refpath->nodeNum = refpath->laneNodeInfos.size(); 
	DEBUG("zlm::get_refpath: refpath->nodeNum  = %d \r\n",refpath->laneNodeInfos.size());
	DEBUG("zlm::get_refpath: node_idx as follow \r\n/");
	for(uint32 node_idx =0; node_idx < refpath->laneNodeInfos.size();node_idx+=2)
	{
		DEBUG("zlm::get_refpath: [node_idx_x, node_idx_y, node_idx_heading] = %f %f %f\r\n" ,refpath->laneNodeInfos[node_idx].x, \
			refpath->laneNodeInfos[node_idx].y ,refpath->laneNodeInfos[node_idx].heading);
	}

}

///***************************************************************************************************
//* 功  能: 障碍物筛选
//* 处  理： 根据当前道路宽度对frenet坐标系下的障碍物进行目标筛选
//* 输  入: objects_list      原始障碍物列表
//*		   hdMapTrajectory   来自地图的轨迹点坐标
//		   hdmapInfos         来自地图的目标点及地图坐标系信息
//		   globePLane         来自地图的全局规划车道信息
//		   localPLanne        不在全局轨迹上时，来自地图的当前所在车道及下一段车道信息
//		   localTrajectory    来自局部轨迹规划的信息
//* 输  出: 筛选后的障碍物
//***************************************************************************************************/
void decision::ObjDetect(int onpath, hdMapTrajectory *Trajectory, Dt_RECORD_HdmapInfo *hdmapInfos,\
     Dt_RECORD_HdmapFrontPLane *globePLane, Dt_RECORD_HdmapLocalLane *localPLanne,  \
     Dt_RECORD_LocalizationInfo *localInfos, Dt_RECORD_EnvModelInfos *envModelInfo, EgoConfigPara ego_config, objSecList *selectObj)
{
	memset(selectObj,0,sizeof(objSecList));
	double obstalce_cipv_s =30;
	double obstalce_cipv_d =0;
	uint32 obstalce_cipv_idx =0;

	double laneWidth = 0;
    if(onpath)   // HDMAP
		laneWidth = globePLane->PlanSeg[0].Lane[0].lane_width;
	else
		laneWidth = localPLanne->LocalLane[0].lane_width;

	if(envModelInfo->Lanes.LaneLines->confidence > 0.5)  // ENV  laneInfo
		laneWidth = envModelInfo->Lanes.width;

	if(laneWidth ==0)
		laneWidth = 2.8; // default lane width

	laneInfo refpath; //define refpath
	memset(&refpath, 0, sizeof(laneInfo));
	decision::get_refpath(onpath, Trajectory, &refpath);
	// decision::convert_flat_to_vehicle(localInfos, &refpath);  // get refpath in vehicle coordinate

	// get refpath_x, refpath_y
	std::vector<double> refpath_x;
	std::vector<double> refpath_y;
	for(uint32 point_idx = 0; point_idx < refpath.nodeNum; ++point_idx)
	{
		refpath_x.push_back(refpath.laneNodeInfos[point_idx].x);
		refpath_y.push_back(refpath.laneNodeInfos[point_idx].y);
	}
	
	// calculate ego vehicle pos's s and d
	std::vector<double> loc_sd = decision::getFrenet(0,0,0, refpath_x, refpath_y);
			
	// std::cout << "zlm::ObjDetect: loc_sd :" << loc_sd[0] <<" " << loc_sd[1] <<std::endl;
	
	DEBUG("OBJ SELECTION----> envModelInfo->obstacle_num= %d\r\n                     ", envModelInfo->obstacle_num);
	DEBUG("OBJ SELECTION----> lane_width / 2= %f\r\n                     ", laneWidth / 2);
	//  iterater all obstacle to select CIPV
	for (uint32 obj_idx = 0; obj_idx < envModelInfo->obstacle_num; obj_idx++)
	// uint32 obj_idx =3;
	{	
		
		// std::vector<double> obj_sd = decision::getFrenet(envModelInfo->Obstacles[obj_idx].pos_x, -envModelInfo->Obstacles[obj_idx].pos_y, \
		// 	envModelInfo->Obstacles[obj_idx].heading, refpath_x, refpath_y); 
		std::vector<double> obj_sd = decision::getFrenet(envModelInfo->Obstacles[obj_idx].pos_x, envModelInfo->Obstacles[obj_idx].pos_y, \
			0, refpath_x, refpath_y); 
		// std::cout << "zlm::ObjDetect: obj_posx obj_posy :" << envModelInfo->Obstacles[obj_idx].pos_x \
		// 	<<" " << envModelInfo->Obstacles[obj_idx].pos_y <<std::endl;
		// obj_sd[0] = obj_sd[0] - loc_sd[0]; // update obstacle property s, which make it start from ego vehicle pos
		DEBUG("OBJ SELECTION----> ObjDetect: veh_loc_x , y =%f %f \r\n                     ", loc_sd[0], loc_sd[1]);
		double obj_s = obj_sd[0];
		double obj_d = obj_sd[1];
		// std::cout << "zlm::ObjDetect: obj_sd :" << obj_s <<" " << obj_d <<std::endl;
		DEBUG("OBJ SELECTION----> ObjDetect: obj[i]_x       = %f\r\n", envModelInfo->Obstacles[obj_idx].pos_x);
		DEBUG("OBJ SELECTION----> ObjDetect: obj[i]_y       = %f\r\n", envModelInfo->Obstacles[obj_idx].pos_y);
		DEBUG("OBJ SELECTION----> ObjDetect: obj[i]_heading = %f\r\n", envModelInfo->Obstacles[obj_idx].heading);

		DEBUG("OBJ SELECTION----> ObjDetect: obj[i]_s       = %f\r\n", obj_s);
		DEBUG("OBJ SELECTION----> ObjDetect: obj[i]_d       = %f\r\n", obj_d);

		if(obj_d > -laneWidth/2 && obj_d < laneWidth/2 &&  obj_s > 3 &&  obj_s < obstalce_cipv_s)
		{
			obstalce_cipv_s = obj_s; // update min s
			obstalce_cipv_d = obj_d; // update min s
			obstalce_cipv_idx = obj_idx;
			selectObj->frontMid.postion = 1;
		}
	}

	if(selectObj->frontMid.postion == 1)
	{
		selectObj->frontMid.obj.id          = envModelInfo->Obstacles[obstalce_cipv_idx].id;
		selectObj->frontMid.obj.type        = envModelInfo->Obstacles[obstalce_cipv_idx].type;
		selectObj->frontMid.obj.pos_x       = envModelInfo->Obstacles[obstalce_cipv_idx].pos_x;
		selectObj->frontMid.obj.pos_y       = envModelInfo->Obstacles[obstalce_cipv_idx].pos_y;
		selectObj->frontMid.obj.heading       = envModelInfo->Obstacles[obstalce_cipv_idx].heading;
		selectObj->frontMid.obj.abs_speed_x = envModelInfo->Obstacles[obstalce_cipv_idx].abs_speed_x;
		selectObj->frontMid.obj.abs_speed_y = envModelInfo->Obstacles[obstalce_cipv_idx].abs_speed_y;
		selectObj->frontMid.obj.rel_speed_x = envModelInfo->Obstacles[obstalce_cipv_idx].rel_speed_x;
		selectObj->frontMid.obj.rel_speed_y = envModelInfo->Obstacles[obstalce_cipv_idx].rel_speed_y;
		selectObj->frontMid.obj.s = obstalce_cipv_s;
		selectObj->frontMid.obj.d = obstalce_cipv_d;
	}

	// if(selectObj->frontMid.postion == 1) // ONLY print with success selection 
	{
	
		DEBUG("OBJ SELECTION----> onpath= %d\r\n                     ", onpath);
		DEBUG("OBJ SELECTION----> selectObj->frontMid.postion= %d \r\n", selectObj->frontMid.postion);
		DEBUG("OBJ SELECTION----> selectObj->frontMid.obj.id = %d \r\n", selectObj->frontMid.obj.id);
		DEBUG("OBJ SELECTION----> selectObj->frontMid.obj.s = %f \r\n", selectObj->frontMid.obj.s);
		DEBUG("OBJ SELECTION----> selectObj->frontMid.obj.d = %f \r\n", selectObj->frontMid.obj.d);
		DEBUG("OBJ SELECTION----> selectObj->frontMid.obj.pos_x = %f \r\n", selectObj->frontMid.obj.pos_x);
		DEBUG("OBJ SELECTION----> selectObj->frontMid.obj.pos_y = %f \r\n", selectObj->frontMid.obj.pos_y);
		DEBUG("OBJ SELECTION----> selectObj->frontMid.obj.heading = %f \r\n", selectObj->frontMid.obj.heading);
		DEBUG("OBJ SELECTION----> selectObj->frontMid.obj.rel_speed_x = %f \r\n", selectObj->frontMid.obj.rel_speed_x);
		DEBUG("OBJ SELECTION----> selectObj->frontMid.obj.rel_speed_y = %f \r\n", selectObj->frontMid.obj.rel_speed_y);
		DEBUG("OBJ SELECTION----> selectObj->frontMid.obj.abs_speed_x  = %f \r\n", selectObj->frontMid.obj.abs_speed_x);
		DEBUG("OBJ SELECTION----> selectObj->frontMid.obj.type  = %d \r\n", selectObj->frontMid.obj.type);
	}
}