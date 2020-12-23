#ifndef __VMCPARKING_H__
#define __VMCPARKING_H__
#include "tcpsocket.h"
typedef unsigned char         boolean;       /*        TRUE .. FALSE           */

typedef signed char           sint8;         /*        -128 .. +127            */
typedef unsigned char         uint8;         /*           0 .. 255             */
typedef signed short          sint16;        /*      -32768 .. +32767          */
typedef unsigned short        uint16;        /*           0 .. 65535           */
//typedef signed long           sint32;        /* -2147483648 .. +2147483647     */
//typedef unsigned long         uint32;        /*           0 .. 4294967295      */

typedef signed long slong32;
typedef unsigned long ulong32;

typedef signed char           sint8_least;   /* At least 7 bit + 1 bit sign    */
typedef unsigned char         uint8_least;   /* At least 8 bit                 */
typedef signed short          sint16_least;  /* At least 15 bit + 1 bit sign   */
typedef unsigned short        uint16_least;  /* At least 16 bit                */
//typedef signed long           sint32_least;  /* At least 31 bit + 1 bit sign   */
//typedef unsigned long         uint32_least;  /* At least 32 bit                */
typedef signed long slong32_least;
typedef unsigned long ulong32_least;

typedef signed    long long   sint64;   /* -9223372036854775808 .. 9223372036854775807      */
typedef unsigned  long long   uint64;   /*                    0 .. 18446744073709551615     */

typedef float                 float32;
typedef double                float64;
typedef unsigned  long long Idt_uint64;
//=============================================
#pragma pack(2)


typedef struct _Dt_RECORD_VehicleSpeed
{
	float32 ESP_VehSpd;
	float32 ESP_WhlSpd_LF;
	uint8 ESP_WhlMovgDir_LF;
	float32 ESP_WhlSpd_RF;
	uint8 ESP_WhlMovgDir_RF;
	float32 ESP_WhlSpd_RL;
	uint8 ESP_WhlMovgDir_RL;
	float32 ESP_WhlSpd_RR;
	uint8 ESP_WhlMovgDir_RR;
	float32 YRS_LgtAcce;
	float32 YRS_LatAcce;
	float32 YRS_YawRate;
	float32 ESP_RoadSlop;
	friend ostream& operator<<(ostream& ss, const struct _Dt_RECORD_VehicleSpeed& token)
	{
		ss << "ESP_VehSpd:" << token.ESP_VehSpd << "|ESP_WhlSpd_LF:" << token.ESP_WhlSpd_LF
			<< "|ESP_WhlMovgDir_LF:" << token.ESP_WhlMovgDir_LF << "|ESP_WhlSpd_RF:" << token.ESP_WhlSpd_RF
			<< "|ESP_WhlMovgDir_RF:" << token.ESP_WhlMovgDir_RF << "|ESP_WhlSpd_RL:" << token.ESP_WhlSpd_RL
			<< "|ESP_WhlMovgDir_RL:" << token.ESP_WhlMovgDir_RL << "|ESP_WhlSpd_RR:" << token.ESP_WhlSpd_RR
			<< "|ESP_WhlMovgDir_RR:" << token.ESP_WhlMovgDir_RR << "|YRS_LgtAcce:" << token.YRS_LgtAcce
			<< "|YRS_LatAcce:" << token.YRS_LatAcce << "|YRS_YawRate:" << token.YRS_YawRate
			<< "|ESP_RoadSlop:" << token.ESP_RoadSlop;
		return ss;
	}
} Dt_RECORD_VehicleSpeed;

typedef struct _Dt_RECORD_ParkTrajectoryPoint
{
	float32 x;
	float32 y;
	float32 w;
	float32 curvature;
	float32 v;
	ulong32 d;
	ulong32 segment_id;
	float32 distance;
	friend ostream& operator<<(ostream& ss, const struct _Dt_RECORD_ParkTrajectoryPoint& token)
	{
		ss << "x:" << token.x << "|y:" << token.y
			<< "|w:" << token.w << "|curvature:" << token.curvature
			<< "|v:" << token.v << "|d:" << token.d
			<< "|segment_id:" << token.segment_id << "|distance:" << token.distance;
		return ss;
	}
} Dt_RECORD_ParkTrajectoryPoint;

typedef Dt_RECORD_ParkTrajectoryPoint Dt_ARRAY_500_ParkTrajectoryPoint[500];
typedef ulong32 Dt_ARRAY_7_SegmentPointNum[7];

typedef struct _Dt_RECORD_ParkingPosition
{
	float32 x;
	float32 y;
	float32 w;
	friend ostream& operator<<(ostream& ss, const struct _Dt_RECORD_ParkingPosition& token)
	{
		ss << "x:" << token.x << "|y:" << token.y
			<< "|w:" << token.w;
		return ss;
	}
} Dt_RECORD_ParkingPosition;

typedef struct _Dt_RECORD_ParkTrajectoryInfos
{
	ulong32 length;
	Dt_ARRAY_500_ParkTrajectoryPoint ParkTrajectoryPoint;
	uint8 segment_num;
	Dt_ARRAY_7_SegmentPointNum SegmentPointNum;
	uint8 parking_sucess;
	boolean aeb;
	friend ostream& operator<<(ostream& ss, const struct _Dt_RECORD_ParkTrajectoryInfos& token)
	{
		ss << "length:" << token.length << "|ParkTrajectoryPoint:" << token.ParkTrajectoryPoint
			<< "|segment_num:" << token.segment_num << "|SegmentPointNum:" << token.SegmentPointNum
			<< "|parking_sucess:" << token.parking_sucess << "|aeb:" << token.aeb;
		return ss;
	}
} Dt_RECORD_ParkTrajectoryInfos;

typedef struct _Dt_RECORD_PakingPathStatus
{
	uint8 pakingpath_status;
	boolean obstacle_flag;
	friend ostream& operator<<(ostream& ss, const struct _Dt_RECORD_PakingPathStatus& token)
	{
		ss << "pakingpath_status:" << token.pakingpath_status << "|obstacle_flag:" << token.obstacle_flag;
		return ss;
	}
} Dt_RECORD_PakingPathStatus;

typedef struct _Dt_RECORD_ParkingStatus
{
	boolean parking_status;
	friend ostream& operator<<(ostream& ss, const struct _Dt_RECORD_ParkingStatus& token)
	{
		ss << "parking_status:" << token.parking_status;
		return ss;
	}
} Dt_RECORD_ParkingStatus;

typedef struct _Dt_RECORD_ParkingInfo
{
	uint8 ParkInState;
	uint8 ParkOutState;
	friend ostream& operator<<(ostream& ss, const struct _Dt_RECORD_ParkingInfo& token)
	{
		ss << "ParkInState:" << token.ParkInState << "|ParkOutState:" << token.ParkOutState;
		return ss;
	}
} Dt_RECORD_ParkingInfo;

typedef struct _Dt_RECORD_ParkingResult
{
	Dt_RECORD_ParkingPosition ParkingPosition;
	ulong32 pk_track_over_flag;
	friend ostream& operator<<(ostream& ss, const struct _Dt_RECORD_ParkingResult& token)
	{
		ss << "ParkingPosition:" << token.ParkingPosition << "|pk_track_over_flag:" << token.pk_track_over_flag;
		return ss;
	}
} Dt_RECORD_ParkingResult;

typedef struct _Dt_RECORD_ControlOutput
{
	float32 APA_AxReq;
	float32 APA_StrWhlAgReq;
	uint8   APA_ShiftReq;
	float32 APA_VLCAxTarUpprCmftBand;
	float32 APA_VLCAxTarLowrCmftBand;
	float32 APA_VLCAxTarUpprLim;
	float32 APA_VLCAxTarLowrLim;
	boolean APA_BrkPref;
	uint8   APA_VLCShudwnReq;
	uint8   APA_DstToStop;
	boolean APA_EmgyBrkReq;
	boolean APA_VehSecureRels;
	boolean APA_VehSecureHold;
	boolean APA_VehStandstillReq;
	boolean APA_DrvOffReq;
	boolean APA_PrefillReq;
	friend ostream& operator<<(ostream& ss, const struct _Dt_RECORD_ControlOutput& token)
	{
		ss << "APA_AxReq:" << token.APA_AxReq << "|APA_StrWhlAgReq:" << token.APA_StrWhlAgReq
			<< "|APA_ShiftReq:" << token.APA_ShiftReq << "|APA_VLCAxTarUpprCmftBand:" << token.APA_VLCAxTarUpprCmftBand
			<< "|APA_VLCAxTarLowrCmftBand:" << token.APA_VLCAxTarLowrCmftBand << "|APA_VLCAxTarUpprLim:" << token.APA_VLCAxTarUpprLim
			<< "|APA_VLCAxTarLowrLim:" << token.APA_VLCAxTarLowrLim << "|APA_BrkPref:" << token.APA_BrkPref
			<< "|APA_VLCShudwnReq:" << token.APA_VLCShudwnReq << "|APA_DstToStop:" << token.APA_DstToStop
			<< "|APA_EmgyBrkReq:" << token.APA_EmgyBrkReq << "|APA_VehSecureRels:" << token.APA_VehSecureRels
			<< "|APA_VehSecureHold:" << token.APA_VehSecureHold << "|APA_VehStandstillReq:" << token.APA_VehStandstillReq
			<< "|APA_DrvOffReq:" << token.APA_DrvOffReq << "|APA_PrefillReq:" << token.APA_PrefillReq;
		return ss;
	}
} Dt_RECORD_ControlOutput;
#pragma pack()
#endif