#pragma once
#include<vector>
#include "Rte_Type.h"


//// FROM MOTION_PLAN.H  ////
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
	float64 pos_s;  // modified
    float64 pos_d;  // add
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





struct objFrame
{
    int frame_num;
    std::vector<Obj_sel> obj;
};

struct Node
{
    double flat_X;
    double flat_Y;
    double Heading;
    double car_x;
    double car_y;
    double s;
    double d;

};

struct Path
{
    int id;
    std::vector<Node> data;
};

struct Location
{
    double flat_X;
    double flat_Y;
    double Heading;
    double car_x;
    double car_y;
    double s;
    double d;
};
