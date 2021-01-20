#pragma once
#include<iostream>
#include "Rte_Type.h"
typedef struct Obj_sel
{
	uint32 id;
	uint32 type;
	uint32 confidence_state;
	uint32 state;
	uint32 lane_no;
	uint32 CIPV_flag;
	uint32 valid_time;
	float64 pos_y;
	float64 pos_x;
	float64 rel_speed_y;
	float64 rel_speed_x;
	float64 rel_acc_y;
	float64 rel_acc_x;
	float64 abs_speed_y;
	float64 abs_speed_x;
	float64 abs_acc_y;
	float64 abs_acc_x;
	float64 heading;
	float64 length;
	float64 width;
	float64 height;
	float64 s;
    float64 d;
}Obj_sel;
typedef struct _objSec_
{
	int postion;           //�ϰ���λ�ã�0����û�У�1������
	Obj_sel obj;
}objSec;

typedef struct _objSecList_
{
	objSec frontMid;//��ǰ���ϰ���
	objSec frontMidLeft;//��������ǰ���ϰ���
	objSec frontMidRight;//��������ǰ���ϰ���
	objSec frontLeft;//�����󳵵�ǰ���ϰ���
	objSec frontRight;//�����ҳ���ǰ���ϰ���
	objSec rearMid;//�����ϰ���
	objSec rearLeft;//����ϰ���
	objSec rearRight;//�Һ��ϰ���
	objSec objSecAEB;

	//��̬�ϰ���
	objSec staticFrontMid;//��ǰ����̬�ϰ���
	objSec staticFrontMidLeft;//��ǰ����̬�ϰ���
	objSec staticFrontMidRight;//��ǰ����̬�ϰ���
}objSecList;


/*通用的决策信息.*/
typedef struct _decision_info_
{
	unsigned int decision_command;   //1：巡航；2：跟车；3：左变道；4：右变道；5：左避障；6：右避障；7：停车；8：AEB；9：右转大曲率；10：左转大曲率
	float velocity;					//期望速度限幅值
	unsigned int lamp;				//0：不亮；1：右转灯；2：左转灯;3:双闪灯
	// trajectory_point stop_point;	//停车点坐标
	// object SelectedObstacles;		//20200326,wsy,增加决策发出的筛选后的障碍物
	unsigned int stopCmd;             //决策发给控制的停车指令   20200410
}decision_info;						//行为决策输出