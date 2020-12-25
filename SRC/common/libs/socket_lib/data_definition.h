#ifndef __DATA_DEFINITION_H__
#define __DATA_DEFINITION_H__
#include "../Rte_Type.h"
#include "VMCParking.h"
#include "tcpsocket.h"
typedef struct mdata
{
	Dt_RECORD_LocalizationInfo my_localizationInfo;
	Dt_RECORD_TrajectoryPointsInfos my_trajectoryPointsInfos;
	Dt_RECORD_ParkStartInfos my_ParkStartInfos;

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
}mdata;

#pragma pack(2)
typedef struct
{
	Dt_RECORD_ParkTrajectoryInfos parkTrajectoryInfos;
	Dt_RECORD_PakingPathStatus 	pakingPathStatus;
	Dt_RECORD_ParkingStatus   	parkingStatus;
	Dt_RECORD_ParkingInfo 		parkingInfo;
	Dt_RECORD_ParkingResult 		parkingResult;
	Dt_RECORD_ControlOutput 		controlOutput;
	Dt_RECORD_VehicleInfo         vehicleInfo;
	Dt_RECORD_VehicleSpeed        vehicleSpeed;
	uint64                        timeStamp;
} Dt_RECORD_VmcParkingToPc;
#pragma pack()

typedef struct
{
	//input data
	Dt_RECORD_VslamInput G_VslamInput;
	Dt_RECORD_AvpCMD G_AvpCMD;
	Dt_RECORD_AvpParkCMD AvpParkCMD;//����
	Dt_RECORD_AvpCallCMD G_AvpCal1CMD2map; //ȡ��
	Dt_RECORD_PlanningFlag G_PlanningFlag;//�ع滮��־ 
	Dt_RECORD_SysMgtStatus G_SysMgtStatus;
	//output data
	Dt_RECORD_HdMapParkingPosition G_HdMapParkingPosition; 
	Dt_RECORD_GlobalPathStatus G_GlobalPathStatus;
	Dt_RECORD_HdMapStatus G_HdMapStatus;
	Dt_RECORD_HdmapState G_HdmapState;
	Dt_RECORD_HdmapInfo G_HdmapInfo;
	Dt_RECORD_HdmapFrontPLane G_FrontPLane;
	Dt_RECORD_HdmapLocalLane G_LocalLane;

	Dt_RECORD_MapInfo G_Map2slamInto;
	Dt_RECORD_ParkInfos G_ParkInfos;
	Dt_RECORD_LaneHeading G_LaneHeading;
}HdmapToPc_data;




void tc397_data_deserialization(TCPClient& sclient, Dt_RECORD_VmcParkingToPc &data)
{
	size_t ret;
	string msg_buf;
	//cout << "sizeof(Dt_RECORD_VmcParkingToPc) = " << sizeof(Dt_RECORD_VmcParkingToPc) << endl;
	ret = sclient.receive(msg_buf, 16036);
	data_deserialization(msg_buf, data.parkTrajectoryInfos);

	ret = sclient.receive(msg_buf, 2);
	data_deserialization(msg_buf, data.pakingPathStatus);

	ret = sclient.receive(msg_buf, 2);
	data_deserialization(msg_buf, data.parkingStatus);

	ret = sclient.receive(msg_buf, 4);
	data_deserialization(msg_buf, data.parkingInfo);

	ret = sclient.receive(msg_buf, 16);
	data_deserialization(msg_buf, data.parkingResult);

	ret = sclient.receive(msg_buf, 36);
	data_deserialization(msg_buf, data.controlOutput);


	//��Ҫ��������
	ret = sclient.receive(msg_buf, 8);
	data_deserialization(msg_buf, data.vehicleInfo.PDCU_0x214_time_stamp );
	ret = sclient.receive(msg_buf, 4);
	data_deserialization(msg_buf, data.vehicleInfo.PDCU_GearSig);
	ret = sclient.receive(msg_buf, 8);
	data_deserialization(msg_buf, data.vehicleInfo.EPS_0x1C2_time_stamp);
	ret = sclient.receive(msg_buf, 4);
	data_deserialization(msg_buf, data.vehicleInfo.EPS_SteerWhlAgSig);
	ret = sclient.receive(msg_buf, 4);
	data_deserialization(msg_buf, data.vehicleInfo.EPS_SteerWhlRotSpd);
	ret = sclient.receive(msg_buf, 4);
	data_deserialization(msg_buf, data.vehicleInfo.EPS_SteerWhlRotSpdDir);
	ret = sclient.receive(msg_buf, 8);
	data_deserialization(msg_buf, data.vehicleInfo.ESP_0x125_time_stamp);
	ret = sclient.receive(msg_buf, 1);
	data_deserialization(msg_buf, data.vehicleInfo.ESP_WhlOdoEdges_FL);
	ret = sclient.receive(msg_buf, 1);
	data_deserialization(msg_buf, data.vehicleInfo.ESP_WhlOdoEdges_FR);
	ret = sclient.receive(msg_buf, 1);
	data_deserialization(msg_buf, data.vehicleInfo.ESP_WhlOdoEdges_RL);
	ret = sclient.receive(msg_buf, 1);
	data_deserialization(msg_buf, data.vehicleInfo.ESP_WhlOdoEdges_RR);
	
	ret = sclient.receive(msg_buf, 44);
	data_deserialization(msg_buf, data.vehicleSpeed);

	ret = sclient.receive(msg_buf, sizeof(uint64));
	data_deserialization(msg_buf, data.timeStamp);
}

void socket_data_deserialization(TCPClient& sclient, mdata& data, unsigned int& id, unsigned long long& timestamp)
{
	string msg;
	size_t ret = sclient.receive(msg, id, timestamp);
	data_deserialization(msg, data);
}

void socket_data_deserialization(TCPClient& sclient, HdmapToPc_data& data, unsigned int& id, unsigned long long& timestamp)
{
	string msg;
	size_t ret = sclient.receive(msg, id, timestamp);
	data_deserialization(msg, data);
}

void socket_data_deserialization(TCPClient& sclient, DecisionToPC& data, unsigned int& id, unsigned long long& timestamp)
{
	string msg;
	size_t ret = sclient.receive(msg, id, timestamp);
	std::cout << "mid" <<std::endl;
	data_deserialization(msg, data);
	std::cout << "des succ" <<std::endl;
}

bool command_send(TCPClient& sclient, char command[], size_t count)
{
	cout << "command is sending..." << endl;
	string msg;
	msg.assign(command, count);
	size_t ret = sclient.send(msg);
	if (ret <= 0)
	{
		cout << "command send failed..." << endl;
		return false;
	}
	else
	{
		cout << "command send success..." << endl;
		return true;
	}
}
#endif