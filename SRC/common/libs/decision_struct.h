#pragma once
#include <iostream>
#include <vector>
#include "Platform_Types.h"

using namespace std;

#define CARWIDTH     1.9
#define UNFINED 0      //空状态
#define CRUISE 1       //巡航
#define FOLLOW 2       //跟车
#define LEFTCHANGE 3   //左变道
#define RIGHTCHANGE 4  //右变道
#define LEFTAVOIDACE   5   //左避障
#define RIGHTAVOIDACE   6  //右避障
#define STOP   7       //停车
#define AEB   8        //紧急停车
#define PARKING 9    //泊车SDJ20191011
#define SPEEDLIMIT 15  //停车场限速值
#define AMAXLIMIT    //横向加速度限幅值
#define NODE_SIZE 0.1
#define grid_origin_x 0
#define grid_origin_y 0
#define ROUND(a)  (a-(int)a>0.5?((int)a+1):(int)a)
#define GRID_WIDTH  400
#define GRID_HEIGHT 200


 #define __DEBUG__  //需要打印时放开，不需要打印时注释掉

 #ifdef __DEBUG__  
 #define DEBUG(format,...) printf( /*__FILE__*/ "===Line: %05d: " format "===", __LINE__, ##__VA_ARGS__)  

 #else  
 #define DEBUG(format,...)  
 #endif 
/*枚举高精地图道路边界类型*/
enum  boundary_type
{
	UNDEFINED = 0,                           //0未定义
	ROADCURB,								 //路牙
	FENCE,                                  //护栏
	WALL,                                   //墙体
	COLUMN,								//柱体
	ROADSTONE,                            //路基石
	WARN_COLUMN,						//警示柱
	DOT,									//道路边界线（虚线）
	REAL,									//道路边界线（实线）
	DOUBLE_REAL,							//道路边界线（双实线）
	DOUBLE_DOT,						   	//道路边界线（双虚线）
	LEFTDOT_RIGHTREAL,					//道路边界线（左实右虚）
	RIGHTREAL_LEFTDOT,					//道路边界线（右实左虚）
};

/*地图发来的单个轨迹点信息*/
typedef struct _lane_node_{
	float64 x;       //地图坐标系中x坐标
	float64 y;	   //地图坐标系中y坐标
	float64 z;	   //地图坐标系中z坐标
	float64 global_x;	  //全局坐标系中x坐标
	float64 global_y;     //全局坐标系中y坐标
	float64 longitude;    //经度
	float64 latitude;     //纬度
	float64 heading;      //航向
	float64 curvature;    //曲率
	float64 slopev;       //纵向坡度
	float64 s;         //点坐标距离本车的距离
}laneNode;

typedef struct lane_info_
{
	uint32 nodeNum;
	vector<laneNode>laneNodeInfos;
}laneInfo;
/*地图发来的全局轨迹*/
typedef struct _path_info_
{
	uint32 laneNum;          //代表此段车道上是否有相邻车道 1代表无相邻车道 2代表仅有左车道 3代表仅有右车道 4代表左右车道均存在
	laneInfo laneInfos[3];   //左中右车道	0表示中间车道，1表示左车道，2代表右车道
}pathInfo;
typedef struct _hdmap_path_info_
{
	uint32 segNum;       //车道段数量
	pathInfo hdmapPathInfo[20];
}hdmapPathInfos;
typedef struct _hdmap_info_
{
	bool planpath;
	float64 origin_x;
	float64 origin_y;
	float64 origin_z;
	float64 origin_yaw; //地图坐标系与正北方向夹角
	float64		goal_x;                                   //路径规划终点x(车体坐标系下)
	float64     goal_y;									//路径规划终点y
	float64     goal_z;									//路径规划终点z
	float64     goal_yaw;									//路径规划终点航向
	float64     goal_s;                                    //路径规划终点离当前车辆距离

}hdmapGoalInfo;

typedef struct _globalTrajectory_
{
	hdmapPathInfos pathLane[2];            //全局轨迹上的数据两组数据，[0]表示前向轨迹，[1]表示后方轨迹
	hdmapPathInfos localPath[2];                  //不在全局轨迹上，当前所在车道段及下一车道段信息

}hdMapTrajectory;               //来自地图的轨迹信息

//可以用来调参的数据，配置参数结构体
typedef struct _EgoConfigPara
{
	float delta_t;                                          //碰撞检测时，轨迹点的时间间隔
	float traj_detect_t;                                 //用于风险评估的预测轨迹的时间长度,单位:s
	float ego_width;                                    //本车宽度
	float ego_length;                                   //本车长度
	float IMUToHead;                                  //IMU到车头的距离
	float IMUToTail;                                     //IMU到车尾的距离
	float human_collision_dis;                    //本车与行人或非机动车距离小于多少时，视为碰撞
	float dynamic_vehicle_collision_dis;    //本车与动态机动车距离小于多少时，视为碰撞
	float static_vehicle_collision_dis;         //本车与静态机动车距离小于多少时，视为碰撞
	float other_collision_dis;                      //本车与其他障碍物（栅格图中）的距离小于多少时，视为碰撞


	float max_follow_error;                          //跟踪误差欧式距离，单位：m
	float max_yaw_error;                                   //跟踪误差yaw，单位：弧度

	float max_length_left_to_plan;                   //当前轨迹剩余多少长度时进行重规划
	float max_length_lane_left_to_plan;            //当前不是优先车道，剩余多少距离时需要重规划

	float collision_stop_time;                             //停车时间，单位ms

	float max_acc_hard_stop;                          //紧急停车最大减速度，m/s^2
	float system_delay;                                 //系统延时，单位：s
	float  dis_from_ob_stop;                           //停止时，本车距离障碍物的距离

	float min_dis_pick_up_to_lane_begin;      //接客点距离本lane 的起点的最小距离

	float static_vehicle_velocity;                      //定义静止车辆的速度
	float follow_vehicle_velocity;                  // 跟车速度上限，下限是静止车辆的速度

	float min_dis_need_follow;                    //小于此距离判断需要跟车
	float min_yaw_error_need_follow;         //航向角小于这个值时需要跟车
	float follow_time_distance;                   //跟车时距
}EgoConfigPara;

//routeobj
typedef struct
{	
	int obj_mid_flag;
	float64 x_m;
	float64 y_m;
	float64 s_m;
	int obj_lef_flag;
	float64 x_l;
	float64 y_l;
	float64 s_l;
	int obj_rig_flag;
	float64 x_r;
	float64 y_r;
	float64 s_r;
}gridmap_coll_obj;

typedef struct _DecisionToPC
{
	Dt_RECORD_LocalizationInfo my_localizationInfo;
	Dt_RECORD_TrajectoryPointsInfos my_trajectoryPointsInfo;
	Dt_RECORD_ParkStartInfos my_parkStartInfo;

		/*新增数据*/
	Dt_RECORD_ParkingSpace my_parkingPlaceInfo;	   //车位融合模块发来的车位信息
	Dt_RECORD_EnvModelInfos my_envModelInfo;		   //环境建模发来的障碍物交通灯等信息
	Dt_RECORD_AccInfo my_vehicleInfo;				   //can解析模块发来的车辆信息
	Dt_RECORD_AvpCMD my_sysmgtAvpCmdInfo;			   //系统管理模块发来的AVP状态请求信息泊取车指令状态，0：无请求1：代客泊车2：召唤取车
	Dt_RECORD_AvpParkCMD my_avpParkCmd;			   //TBOX发来的AVP目标车位信息
	Dt_RECORD_SysMgtStatus my_sysmgtStatusInfo;	   //系统管理模块发来的系统管理状态信息
	Dt_RECORD_ApaUserReq my_apaUserReqInfo;		   //来自CtapCom模块的信息
	Dt_RECORD_HdmapInfo my_hdmapInfo;				   //高精地图发来的是否在全局路径上等信息
	Dt_RECORD_HdmapFrontPLane my_hdmapFrontPLaneInfo; //高精地图发来的全局路径信息
	Dt_RECORD_HdmapLocalLane my_hdmapLocalLaneInfo;   //当不在全局路径上时，高精地图发来的本车道、相邻车道及下一段车道信息
	Dt_RECORD_GlobalPathStatus my_globalPathStatus;   //高精地图发来的全局路径是否规划成功标志	
}DecisionToPC;

