#ifndef RTE_TYPE_H
#define RTE_TYPE_H

#include "Platform_Types.h"


static char Rte_type_version[] = "v2.8.0";
typedef struct _Dt_RECORD_MvParkingPoint
{
    float64 x;
    float64 y;
} Dt_RECORD_MvParkingPoint;

typedef uint8 Dt_ARRAY_262144_szSegData[262144];

typedef struct _Dt_RECORD_MvImuInfo
{
    uint64 time_stamp;
    float64 GyroX;
    float64 GyroY;
    float64 GyroZ;
    float64 AccX;
    float64 AccY;
    float64 AccZ;
    float32 Temperature;
} Dt_RECORD_MvImuInfo;

typedef uint8 Dt_ARRAY_230400_szImageData[230400];

typedef struct _Dt_RECORD_HppTrajectoryPoint
{
    float32 x;
    float32 y;
    float32 w;
    float32 curvature;
    uint32 d;
    uint32 segment_id;
    float32 distance;
} Dt_RECORD_HppTrajectoryPoint;

typedef struct _Dt_RECORD_LaneLines
{
    uint32 id;
    uint32 confidence;
    uint32 type;
    uint32 color;
    float64 line_width;
    float64 length;
    float64 lane_line_lineC0;
    float64 lane_line_lineC1;
    float64 lane_line_lineC2;
    float64 lane_line_lineC3;
} Dt_RECORD_LaneLines;

typedef uint8 Dt_ARRAY_200_GridInfoX[200];

typedef struct _Dt_RECORD_FrontRadarInfos
{
    float32 MPC_TiStamp;
    boolean MPC_TiStampStsGlbTiBas;
    boolean MPC_TiStampStsSyncToGatewy;
    boolean MPC_TiStampStsTiLeap;
    boolean MPC_TiStampStsTiOut;
    uint16 MPC_measure_Latency;
} Dt_RECORD_FrontRadarInfos;

typedef struct _Dt_RECORD_SurVehicleInfos
{
    uint64 ESP_0x318_time_stamp;
    float32 ESP_VehSpd;
} Dt_RECORD_SurVehicleInfos;

typedef struct _Dt_RECORD_ObjPoint
{
    float64 x;
    float64 y;
    float32 z;
} Dt_RECORD_ObjPoint;

typedef struct _Dt_RECORD_TrafficLights
{
    uint32 id;
    uint32 type;
    uint32 confidence;
    float64 pos_y;
    float64 pos_x;
    float64 pos_z;
    uint32 state;
    uint32 lane_no;
} Dt_RECORD_TrafficLights;

typedef struct _Dt_RECORD_SysMgtStatus
{
    uint8 StateAVP;
    uint8 StateAVPLast;
    uint8 StateAPA;
    uint8 StateAPALast;
    uint8 StateAVM;
    uint8 StatePAS;
} Dt_RECORD_SysMgtStatus;

typedef uint8 Dt_ARRAY_1720320_szSegData[1720320];

typedef struct _Dt_RECORD_FlFrCmrrFreespace
{
    float32 CMRR_FL_FreespaceSectorDst;
    uint8 CMRR_FL_FreespaceSectorConfidence;
    uint8 CMRR_FL_FreespaceSectorKineSt;
} Dt_RECORD_FlFrCmrrFreespace;

typedef struct _Dt_RECORD_RlRrCmrrFreespace
{
    float32 CMRR_RL_FreespaceSectorDst;
    uint8 CMRR_RL_FreespaceSectorConfidence;
    uint8 CMRR_RL_FreespaceSectorKineSt;
} Dt_RECORD_RlRrCmrrFreespace;

typedef struct _Dt_RECORD_HdmapInfo
{
    boolean planpath;
    float64 origin_x;
    float64 origin_y;
    float32 origin_z;
    float32 origin_yaw;
    float32 goal_x;
    float32 goal_y;
    float32 goal_z;
    float32 goal_yaw;
    uint8 goalLaneID;
} Dt_RECORD_HdmapInfo;

typedef struct _Dt_RECORD_MpcObstacles
{
    uint8 MPC_obstacle_type;
    boolean MPC_obstacle_CIPV_Flag;
    uint8 MPC_OBJ_Track_ID;
    float32 MPC_OBJ_Width;
    float32 MPC_OBJ_Height;
    float32 MPC_OBJ_Length;
    float32 MPC_OBJ_HeadingAngle;
    float32 MPC_OBJ_Dy;
    float32 MPC_OBJ_Dx;
    float32 MPC_OBJ_Vy;
    float32 MPC_OBJ_Vx;
    float32 MPC_Obj_XVelAbsolute;
    float32 MPC_Obj_YVelAbsolute;
    float32 MPC_OBJ_Ay;
    float32 MPC_OBJ_Ax;
    float32 MPC_OBJ_Ay_relative;
    float32 MPC_OBJ_Ax_relative;
    uint8 MPC_OBJ_Location;
    uint8 MPC_OBJ_state;
    float32 MPC_Obj_ProbOfExist;
    uint8 MPC_obstacle_state;
    uint8 MPC_obstacle_lane_no;
} Dt_RECORD_MpcObstacles;

typedef struct _Dt_RECORD_AvpParkCMD
{
    uint8 TBOX_AVPSanMod;
    uint32 TBOX_AVPTarPrkID;
    uint8 TBOX_AVPTarZoneID;
} Dt_RECORD_AvpParkCMD;

typedef struct _Dt_RECORD_GlobalPathStatus
{
    uint8 globalpath_status;
    uint8 planing_failed_times;
} Dt_RECORD_GlobalPathStatus;

typedef struct _Dt_RECORD_boundary
{
    uint32 boundaryID;
    uint8 boundary_type;
    uint8 boundary_color;
} Dt_RECORD_boundary;

typedef Dt_RECORD_LaneLines Dt_ARRAY_4_LaneLines[4];

typedef Dt_RECORD_MvParkingPoint Dt_ARRAY_4_MvParkingPoint[4];

typedef Dt_RECORD_boundary Dt_ARRAY_5_boundary[5];

typedef struct _Dt_RECORD_Crosswalk
{
    uint32 ID;
    float32 distance;
} Dt_RECORD_Crosswalk;

typedef Dt_RECORD_ObjPoint Dt_ARRAY_50_ObjPoint[50];

typedef struct _Dt_RECORD_HdmapObj
{
    uint32 objID;
    uint8 type;
    uint8 PointNum;
    Dt_ARRAY_50_ObjPoint ObjPoint;
} Dt_RECORD_HdmapObj;

typedef struct _Dt_RECORD_SurCamFaultStatus
{
    uint8 FrontSurCamFault;
    uint8 LeftSurCamFault;
    uint8 RightSurCamFault;
    uint8 RearSurCamFault;
} Dt_RECORD_SurCamFaultStatus;

typedef struct _Dt_RECORD_TrajectoryPoints
{
    float32 x;
    float32 y;
    float32 x_g;
    float32 y_g;
    float32 t;
    float32 lon;
    float32 lat;
    float32 theta;
    float32 v;
    float32 a;
    float32 kappa;
    float32 slope;
    uint32 lane_no;
    uint32 road_no;
} Dt_RECORD_TrajectoryPoints;

typedef struct _Dt_RECORD_UsscParkingLeft
{
    uint8 USSC_Slotpropety_L;
    uint8 USSC_SlotConfidence_L;
    uint8 USSC_SlotConfidence_Flag_L;
    boolean USSC_SlotupdateOcd_L;
    sint16 USSC_SlotpiontAPosX_L;
    sint16 USSC_SlotpiontAPosY_L;
    sint16 USSC_SlotpiontBPosX_L;
    sint16 USSC_SlotpiontBPosY_L;
    sint16 USSC_SlotpiontCPosX_L;
    sint16 USSC_SlotpiontCPosY_L;
    sint16 USSC_SlotpiontDPosX_L;
    sint16 USSC_SlotpiontDPosY_L;
    uint8 USSC_PrkgSlotSta_L;
} Dt_RECORD_UsscParkingLeft;

typedef struct _Dt_RECORD_ParkingFusionStatus
{
    boolean parking_fusion_status;
} Dt_RECORD_ParkingFusionStatus;

typedef struct _Dt_RECORD_ParkingPoint
{
    float64 x;
    float64 y;
} Dt_RECORD_ParkingPoint;

typedef Dt_RECORD_FlFrCmrrFreespace Dt_ARRAY_72_FlFrCmrrFreespace[72];

typedef struct _Dt_RECORD_ParkId
{
    uint32 parking_id;
} Dt_RECORD_ParkId;

typedef Dt_RECORD_RlRrCmrrFreespace Dt_ARRAY_72_RlRrCmrrFreespace[72];

typedef float32 Dt_ARRAY_8_park_pos[8];

typedef Dt_RECORD_TrafficLights Dt_ARRAY_8_TrafficLights[8];

typedef Dt_ARRAY_200_GridInfoX Dt_ARRAY_80000_ObstacleGridMap[400];

typedef uint8 Dt_ARRAY_921600_szImageData[921600];

typedef struct _Dt_RECORD_AccInfo
{
    uint64 ESP_0x318_time_stamp;
    float32 ESP_VehSpd;
    uint64 YRS_0x242_time_stamp;
    float32 YRS_LgtAcce;
    float32 YRS_LatAcce;
    float32 YRS_YawRate;
} Dt_RECORD_AccInfo;

typedef struct _Dt_RECORD_ApaObj
{
    uint8 APA_ObjDst_FLS;
    uint8 APA_ObjDst_FLC;
    uint8 APA_ObjDst_FLM;
    uint8 APA_ObjDst_FRS;
    uint8 APA_ObjDst_FRC;
    uint8 APA_ObjDst_FRM;
    uint8 APA_ObjDst_RLS;
    uint8 APA_ObjDst_RLC;
    uint8 APA_ObjDst_RLM;
    uint8 APA_ObjDst_RRS;
    uint8 APA_ObjDst_RRC;
    uint8 APA_ObjDst_RRM;
} Dt_RECORD_ApaObj;

typedef struct _Dt_RECORD_ApaParkingInfo
{
    uint8 APA_PrkgMod;
    uint8 APA_OrntnChgAvl;
    uint8 APA_CrtPrkgOrntn;
    uint8 APA_PrkgModChgAvl;
    uint8 APA_PrkgDirChgAvl;
    uint8 APA_UISettingModAvl;
    uint8 APA_PrkgDir;
    uint8 APA_UISettingMod;
    uint8 APA_PrkgSlot_1L;
    uint8 APA_PrkgSlot_2L;
    uint8 APA_PrkgSlot_3L;
    uint8 APA_PrkgSlot_1R;
    uint8 APA_PrkgSlot_2R;
    uint8 APA_PrkgSlot_3R;
    uint16 APA_PrkgSlotCoor_1L;
    uint16 APA_PrkgSlotCoor_2L;
    uint16 APA_PrkgSlotCoor_3L;
    uint16 APA_PrkgSlotCoor_1R;
    uint16 APA_PrkgSlotCoor_2R;
    uint16 APA_PrkgSlotCoor_3R;
    uint8 APA_PrkgSlotSta_1L;
    uint8 APA_PrkgSlotSta_2L;
    uint8 APA_PrkgSlotSta_3L;
    uint8 APA_PrkgSlotSta_1R;
    uint8 APA_PrkgSlotSta_2R;
    uint8 APA_PrkgSlotSta_3R;
} Dt_RECORD_ApaParkingInfo;

typedef struct _Dt_RECORD_ApaUserReq
{
    uint8 EHU_SelKey;
    uint8 EHU_ModSeln;
    uint8 EHU_OrintChgKey;
    uint8 EHU_PerpModChg;
    uint8 EHU_TypChgKey;
    uint8 EHU_CfmResmKey;
    uint8 EHU_CnclKey;
    uint8 EHU_PauseKey;
    uint8 EHU_ResuKey;
    uint8 EHU_RtnStrtKey;
    uint8 EHU_PrkgSlotc;
    uint8 EHU_APAUISettingngMod;
} Dt_RECORD_ApaUserReq;

typedef struct _Dt_RECORD_AvmInfo
{
    uint8 AVAP_ZoomSts;
    uint8 AVAP_ViewSts;
    uint16 AVAP_XCoorFb;
    uint16 AVAP_YCoorFb;
} Dt_RECORD_AvmInfo;

typedef struct _Dt_RECORD_AvmReq
{
    uint8 EHU_3DCrs;
    uint8 EHU_IntegtOpen;
    uint8 EHU_AutCam;
    uint8 EHU_ShowReq;
    boolean EHU_ShowSts;
    uint8 EHU_ViewReq;
    uint8 EHU_OverlayTubeReq;
    uint8 EHU_OverlayDstReq;
    uint8 EHU_SDWAudioReq;
    uint8 EHU_FeaReqBasePlateTex;
    uint8 EHU_FeaReqVehTrans;
    uint8 EHU_FeaReqVehColor;
    uint8 EHU_RestoreDftSettings;
    uint8 EHU_AVMMod;
} Dt_RECORD_AvmReq;

typedef struct _Dt_RECORD_AvmShowInfo
{
    uint8 AVM_UnavlMsgs;
    uint8 AVM_FrntCamSts;
    uint8 AVM_ReCamSts;
    uint8 AVM_LeCamSts;
    uint8 AVM_RiCamSts;
    boolean AVM_FrntViewSts;
    boolean AVM_ReViewSts;
    boolean AVM_LeViewSts;
    boolean AVM_RiViewSts;
    uint8 AVM_ViewMod;
    uint8 AVM_ReqVehColorAck;
    uint8 AVM_3DCrsSts;
    uint8 AVM_IntegtOpenSts;
    uint8 AVM_AutCamSts;
    uint8 AVM_OverlayDistAck;
    uint8 AVM_DynOverlay;
    uint8 AVM_ReqBasPlateTextureAck;
    uint8 AVM_ReqVehTrspysAck;
    boolean AVM_ShowReq;
} Dt_RECORD_AvmShowInfo;

typedef struct _Dt_RECORD_AvmUserReq
{
    uint16 EHU_TouchXCoor;
    uint16 EHU_TouchYCoor;
    uint8 EHU_TouchEveTyp;
    uint8 EHU_ZoomReq;
    uint8 EHU_GesRcntnRes;
} Dt_RECORD_AvmUserReq;

typedef struct _Dt_RECORD_AvpCallCMD
{
    uint8 TBOX_AVPTarPOIID;
} Dt_RECORD_AvpCallCMD;

typedef struct _Dt_RECORD_AvpCMD
{
    uint8 TBOX_AVPModKey;
} Dt_RECORD_AvpCMD;

typedef struct _Dt_RECORD_AVPLocalization
{
    float32 AVP_locationInfo_X0;
    float32 AVP_locationInfo_Y0;
} Dt_RECORD_AVPLocalization;

typedef Dt_RECORD_Crosswalk Dt_ARRAY_10_Crosswalk[10];

typedef struct _Dt_RECORD_LaneNode
{
    float64 hdmap_x;
    float64 hdmap_y;
    float32 hdmap_z;
    float32 heading;
    float32 curvature;
    float32 slopev;
} Dt_RECORD_LaneNode;

typedef Dt_RECORD_LaneNode Dt_ARRAY_100_LaneNode[100];

typedef struct _Dt_RECORD_MarkArrow
{
    uint32 ID;
    uint8 type;
    uint8 color;
    float32 distance;
} Dt_RECORD_MarkArrow;

typedef struct _Dt_RECORD_LaneHeading
{
    float32 origin_yaw;
    float32 road_width;
    float32 heading;
    float64 start_x;
    float64 start_y;
    float32 start_z;
} Dt_RECORD_LaneHeading;

typedef struct _Dt_RECORD_Obstacles
{
    uint32 id;
    uint8 type;
    uint8 confidence_state;
    uint8 state;
    uint32 lane_no;
    uint8 CIPV_flag;
    uint32 valid_time;
    float32 pos_y;
    float32 pos_x;
    float32 rel_speed_y;
    float32 rel_speed_x;
    float32 rel_acc_y;
    float32 rel_acc_x;
    float32 abs_speed_y;
    float32 abs_speed_x;
    float32 abs_acc_y;
    float32 abs_acc_x;
    float32 heading;
    float32 length;
    float32 width;
    float32 height;
    uint32 img_x;
    uint32 img_y;
    uint32 img_width;
    uint32 img_height;
} Dt_RECORD_Obstacles;

typedef struct _Dt_RECORD_FlFrCmrrObject
{
    uint8 CMRR_FR_OBJ_Track_ID;
    uint8 CMRR_FR_OBJ_Type;
    float32 CMRR_FR_OBJ_Width;
    float32 CMRR_FR_OBJ_Height;
    float32 CMRR_FR_OBJ_Length;
    float32 CMRR_FR_OBJ_HeadingAngle;
    float32 CMRR_FR_OBJ_Dy;
    float32 CMRR_FR_OBJ_Dx;
    float32 CMRR_FR_OBJ_Vy;
    float32 CMRR_FR_OBJ_Vx;
    float32 CMRR_FR_OBJ_XVelAbsolute;
    float32 CMRR_FR_OBJ_YVelAbsolute;
    float32 CMRR_FR_OBJ_Ay;
    float32 CMRR_FR_OBJ_Ax;
    uint8 CMRR_FR_OBJ_Location;
    float32 CMRR_FR_OBJ_ExistProb;
    uint8 CMRR_FR_Obj_DistLong_rms;
    uint8 CMRR_FR_Obj_VrelLong_rms;
    uint8 CMRR_FR_Obj_DistLat_rms;
    uint8 CMRR_FR_Obj_VrelLat_rms;
    float32 CMRR_FR_Obj_Orientation_rms;
    uint8 CMRR_FR_OBJ_state;
} Dt_RECORD_FlFrCmrrObject;

typedef Dt_RECORD_FlFrCmrrObject Dt_ARRAY_20_FlFrCmrrObject[20];

typedef struct _Dt_RECORD_LampInfo
{
    uint64 BDCU_0x335_time_stamp;
    boolean BDCU_LeTrunLampOutpCmd;
    boolean BDCU_RiTrunLampOutpCmd;
    boolean BDCU_BrkLampOutpCmd;
} Dt_RECORD_LampInfo;

typedef struct _Dt_RECORD_FrontCamFaultStatus
{
    uint8 FrontCamFaultStatus;
} Dt_RECORD_FrontCamFaultStatus;

typedef struct _Dt_RECORD_FrontFusionStatus
{
    boolean front_fusion_status;
} Dt_RECORD_FrontFusionStatus;

typedef struct _Dt_RECORD_FusionParkingSpace
{
    uint32 id;
    uint32 confidence_state;
    float32 fAngle;
    uint32 Array_Length_Dummy_0;
    Dt_ARRAY_8_park_pos Park;
    float32 park_length;
    float32 park_width;
    float32 park_direction;
    uint32 park_type;
    uint8 park_source;
} Dt_RECORD_FusionParkingSpace;

typedef Dt_RECORD_FusionParkingSpace Dt_ARRAY_16_FusionParkingSpace[16];

typedef struct _Dt_RECORD_FusionVehicleInfo
{
    uint64 ESP_0x318_time_stamp;
    float32 ESP_VehSpd;
    uint64 ESP_0x305_time_stamp;
    float32 ESP_WhlSpd_LF;
    uint8 ESP_WhlMovgDir_LF;
    float32 ESP_WhlSpd_RF;
    uint8 ESP_WhlMovgDir_RF;
    uint64 ESP_0x306_time_stamp;
    float32 ESP_WhlSpd_RL;
    uint8 ESP_WhlMovgDir_RL;
    float32 ESP_WhlSpd_RR;
    uint8 ESP_WhlMovgDir_RR;
} Dt_RECORD_FusionVehicleInfo;

typedef struct _Dt_RECORD_GestureInfo
{
    uint64 time_stamp;
    float64 yaw;
    float64 pitch;
    float64 roll;
    float64 yawrate;
    float64 a_lon;
    float64 a_lat;
    float64 v_lat;
    float64 v_lon;
} Dt_RECORD_GestureInfo;

typedef Dt_RECORD_MarkArrow Dt_ARRAY_10_MarkArrow[10];

typedef struct _Dt_RECORD_PoiObstacle
{
    uint32 ID;
    uint8 type;
    float32 distance;
} Dt_RECORD_PoiObstacle;

typedef Dt_RECORD_PoiObstacle Dt_ARRAY_10_PoiObstacle[10];

typedef struct _Dt_RECORD_HdMapLane
{
    uint8 node_count;
    Dt_ARRAY_100_LaneNode LaneNode;
    uint32 laneID;
    boolean change_lane_flag;
    sint8 lane_NO;
    uint8 lane_position;
    uint8 lane_type;
    float32 lane_width;
    float64 lane_length;
    uint8 lane_way_count;
    uint8 turn_info;
    uint32 lboundaryID;
    uint8 lboundary_type;
    uint8 lboundary_color;
    uint32 rboundaryID;
    uint8 rboundary_type;
    uint8 rboundary_color;
    uint32 road_ID;
    uint8 road_type;
    uint8 road_position;
    uint8 road_direction;
    float64 road_length;
    float32 road_width;
    uint8 lane_count;
    float32 speed_limit;
    uint8 POI_num;
    uint8 obstacle_num;
    uint8 mark_num;
    uint8 arrow_num;
    uint8 crosswalk_num;
    uint8 HGNode_num;
    uint8 lboundary_num;
    uint8 rboundary_num;
    Dt_ARRAY_5_boundary lboundary;
    Dt_ARRAY_5_boundary rboundary;
    float32 hgnode_distance;
    uint32 hgnodeID;
    Dt_ARRAY_10_Crosswalk Crosswalk;
    Dt_ARRAY_10_MarkArrow Arrow;
    Dt_ARRAY_10_PoiObstacle POI;
    Dt_ARRAY_10_PoiObstacle Obstacle;
    Dt_ARRAY_10_MarkArrow Mark;
} Dt_RECORD_HdMapLane;

typedef Dt_RECORD_HdmapObj Dt_ARRAY_100_HdmapObj[100];

typedef struct _Dt_RECORD_MapInfo
{
    uint64 time_stamp;
    float64 lon_x;
    float64 lat_y;
    float32 origin_yaw;
    boolean hdmap_output_flag1;
    boolean hdmap_output_flag2;
    uint8 feature_num;
    Dt_ARRAY_100_HdmapObj HdmapObj;
} Dt_RECORD_MapInfo;

typedef struct _Dt_RECORD_HdMapParkingPosition
{
    boolean in_parking;
    boolean same_parking;
} Dt_RECORD_HdMapParkingPosition;

typedef struct _Dt_RECORD_HdmapState
{
    boolean exit_hdmap;
} Dt_RECORD_HdmapState;

typedef struct _Dt_RECORD_HdMapStatus
{
    uint32 hdmap_status;
} Dt_RECORD_HdMapStatus;

typedef Dt_RECORD_HppTrajectoryPoint Dt_ARRAY_100_HppTrajectoryPoint[100];

typedef struct _Dt_RECORD_HppLocalLane
{
    uint32 node_count;
    uint32 segment_num;
    uint32 Array_Length_Dummy_0;
    Dt_ARRAY_100_HppTrajectoryPoint HppTrajectoryPoint;
    float32 path_terminal_x;
    float32 path_terminal_y;
    float32 driving_terminal_x;
    float32 driving_terminal_y;
} Dt_RECORD_HppLocalLane;

typedef Dt_RECORD_HdMapLane Dt_ARRAY_3_HdMapLane[3];

typedef struct _Dt_RECORD_PlanSeg
{
    uint8 lane_count;
    Dt_ARRAY_3_HdMapLane Lane;
} Dt_RECORD_PlanSeg;

typedef struct _Dt_RECORD_Lanes
{
    Dt_ARRAY_4_LaneLines LaneLines;
    float64 width;
    float64 left_lane_distance;
    float64 right_lane_distance;
} Dt_RECORD_Lanes;

typedef struct _Dt_RECORD_MvUSSObstacleInfo
{
    uint16 USSC_ObjDst_RLC;
    uint16 USSC_ObjDst_RLM;
    uint16 USSC_ObjDst_RRC;
    uint16 USSC_ObjDst_RLS;
    uint16 USSC_ObjDst_RRS;
    uint16 USSC_ObjDst_RRM;
    uint16 USSC_ObjDst_FLS;
    uint16 USSC_ObjDst_FRS;
    uint16 USSC_ObjDst_FLC;
    uint16 USSC_ObjDst_FLM;
    uint16 USSC_ObjDst_FRC;
    uint16 USSC_ObjDst_FRM;
} Dt_RECORD_MvUSSObstacleInfo;

typedef struct _Dt_RECORD_MvMapTrafficInfo
{
    uint32 nType;
    float32 fYDist;
    float32 fXDist;
    float32 fHeight;
} Dt_RECORD_MvMapTrafficInfo;

typedef struct _Dt_RECORD_LocalizationResult
{
    boolean valid;
    float32 x;
    float32 y;
    float32 z;
    float32 qx;
    float32 qy;
    float32 qz;
    float32 qw;
} Dt_RECORD_LocalizationResult;

typedef struct _Dt_RECORD_VslamInput
{
    uint64 time_stamp;
    boolean hdmap_localtion_flag1;
    boolean hdmap_localtion_flag2;
    float64 search_radius1;
    float64 search_radius2;
    float64 lon;
    float64 lat;
    float64 vehicle_x;
    float64 vehicle_y;
    float32 vehicle_z;
    float32 vehicle_yaw;
} Dt_RECORD_VslamInput;

typedef struct _Dt_RECORD_LocationStatus
{
    boolean location_success;
} Dt_RECORD_LocationStatus;

typedef Dt_RECORD_PlanSeg Dt_ARRAY_15_PlanSeg[15];

typedef struct _Dt_RECORD_HdmapFrontPLane
{
    uint8 plan_seg_count;
    Dt_ARRAY_15_PlanSeg PlanSeg;
} Dt_RECORD_HdmapFrontPLane;

typedef Dt_RECORD_MpcObstacles Dt_ARRAY_40_MpcObstacles[40];

typedef struct _Dt_RECORD_MpcInfos
{
    uint8 MPC_obstacle_num;
    uint32 Array_Length_Dummy_0;
    Dt_ARRAY_40_MpcObstacles MpcObstacles;
} Dt_RECORD_MpcInfos;

typedef struct _Dt_RECORD_MvParkingSpace
{
    uint32 nId;
    uint32 nSocre;
    uint32 nValid;
    uint32 nType;
    uint32 nDirection;
    float32 fAngle;
    Dt_ARRAY_4_MvParkingPoint tPoint;
    float64 fLength;
    float64 fWidth;
} Dt_RECORD_MvParkingSpace;

typedef struct _Dt_RECORD_MvGpsInfo
{
    uint64 EHU_0x525_time_stamp;
    float64 EHU_GPSLati;
    float64 EHU_GPSLongi;
    sint32 TBOX_PosngSysLati;
    sint32 TBOX_PosngSysLongi;
} Dt_RECORD_MvGpsInfo;

typedef Dt_RECORD_MvMapTrafficInfo Dt_ARRAY_24_MvMapTrafficInfo[24];

typedef struct _Dt_RECORD_MvTrafficInfo
{
    uint32 nTrafficNum;
    uint32 nReserved;
    Dt_ARRAY_24_MvMapTrafficInfo tMapTrafficInfo;
} Dt_RECORD_MvTrafficInfo;

typedef Dt_RECORD_MvParkingSpace Dt_ARRAY_16_MvParkingSpace[16];

typedef struct _Dt_RECORD_MvFishEyeToLocationData
{
    uint32 nFrameIndex;
    uint64 time_stamp;
    uint32 nParkingSpaceNum;
    Dt_ARRAY_16_MvParkingSpace tParkingSpace;
    uint32 nImageWidth;
    uint32 nImageHeight;
    Dt_ARRAY_921600_szImageData szImageData;
    uint32 nSegDataWidth;
    uint32 nSegDataHeight;
    Dt_ARRAY_1720320_szSegData szSegData;
} Dt_RECORD_MvFishEyeToLocationData;

typedef Dt_RECORD_Obstacles Dt_ARRAY_64_Obstacles[64];

typedef struct _Dt_RECORD_MvFishEyeToSurFusionData
{
    uint32 nFrameIndex;
    uint64 time_stamp;
    uint32 nObstacleNum;
    Dt_RECORD_Lanes Lanes;
    Dt_ARRAY_64_Obstacles Obstacles;
    uint32 nSegDataWidth;
    uint32 nSegDataHeight;
    Dt_ARRAY_1720320_szSegData szSegData;
} Dt_RECORD_MvFishEyeToSurFusionData;

typedef struct _Dt_RECORD_TrafficMarkings
{
    uint32 id;
    uint32 type;
    uint32 confidence;
    uint32 lane_no;
    float64 pos_y;
    float64 pos_x;
} Dt_RECORD_TrafficMarkings;

typedef struct _Dt_RECORD_SteerInfo
{
    uint64 EPS_0x1C2_time_stamp;
    float32 EPS_SteerWhlAgSig;
    uint64 PDCU_0x214_time_stamp;
    uint8 PDCU_GearSig;
} Dt_RECORD_SteerInfo;

typedef struct _Dt_RECORD_EnvModelObstaclesInfos
{
    uint32 obstacle_num;
    uint32 Array_Length_Dummy_0;
    Dt_ARRAY_64_Obstacles Obstacles;
} Dt_RECORD_EnvModelObstaclesInfos;

typedef struct _Dt_RECORD_SLAMLocalizationInfo
{
    uint64 time_stamp;
    Dt_RECORD_LocalizationResult LocalizationResult;
    float64 yaw;
    float64 pitch;
    float64 roll;
} Dt_RECORD_SLAMLocalizationInfo;

typedef struct _Dt_RECORD_ParkInfo
{
    uint32 parkID;
    uint8 park_type;
    float64 park_x1;
    float64 park_y1;
    float64 park_x2;
    float64 park_y2;
    float64 park_x3;
    float64 park_y3;
    float64 park_x4;
    float64 park_y4;
    float64 park_x5;
    float64 park_y5;
    float32 park_radius;
} Dt_RECORD_ParkInfo;

typedef Dt_RECORD_ParkInfo Dt_ARRAY_72_ParkInfo[72];

typedef struct _Dt_RECORD_ParkInfos
{
    uint64 time_stamp;
    float32 origin_yaw;
    uint8 park_num;
    Dt_ARRAY_72_ParkInfo ParkInfo;
} Dt_RECORD_ParkInfos;

typedef Dt_RECORD_ParkingPoint Dt_ARRAY_64_ParkingPoint[64];

typedef struct _Dt_RECORD_SurViewParkingSpace
{
    uint32 id;
    uint32 confidence;
    uint32 valid_state;
    uint32 park_type;
    float64 park_direction;
    float32 fAngle;
    Dt_ARRAY_64_ParkingPoint ParkingPoint;
    float64 park_length;
    float64 park_width;
} Dt_RECORD_SurViewParkingSpace;

typedef struct _Dt_RECORD_ParkingRefPos
{
    float32 parking_length;
    float32 parking_depth;
    uint8 parking_type;
    uint8 park_direction;
    float32 start_point_x;
    float32 start_point_y;
    float32 start_point_heading;
    float32 line_fir_point_x;
    float32 line_fir_point_y;
    float32 line_sec_point_x;
    float32 line_sec_point_y;
    float32 fr_point_x;
    float32 fr_point_y;
    float32 rr_point_x;
    float32 rr_point_y;
    float32 rl_point_x;
    float32 rl_point_y;
    float32 fl_point_x;
    float32 fl_point_y;
} Dt_RECORD_ParkingRefPos;

typedef struct _Dt_RECORD_ParkingSpace
{
    uint8 park_num;
    uint32 Array_Length_Dummy_0;
    Dt_ARRAY_16_FusionParkingSpace FusionParkingSpace;
} Dt_RECORD_ParkingSpace;

typedef struct _Dt_RECORD_ParkStartInfos
{
    uint32 park_start_flag;
    Dt_RECORD_ParkingRefPos ParkingRefPos;
} Dt_RECORD_ParkStartInfos;

typedef struct _Dt_RECORD_PathPlannerInfo
{
    boolean PathPlannerParkStartFlag;
    boolean PathPlannerCallTerminalFlag;
    boolean no_parking;
    uint8 trajectory_status;
    boolean obstacle_flag;
    uint32 light;
    uint8 find_park_type;
} Dt_RECORD_PathPlannerInfo;

typedef struct _Dt_RECORD_PathPlannerStatus
{
    boolean pathplanner_status;
} Dt_RECORD_PathPlannerStatus;

typedef struct _Dt_RECORD_PlanningFlag
{
    boolean planning_hdmap_flag;
} Dt_RECORD_PlanningFlag;

typedef Dt_RECORD_PlanSeg Dt_ARRAY_5_PlanSeg[5];

typedef struct _Dt_RECORD_HdmapLocalLane
{
    uint8 lane_count;
    Dt_ARRAY_3_HdMapLane LocalLane;
    uint8 next_seg_count;
    Dt_ARRAY_5_PlanSeg NextSeg;
} Dt_RECORD_HdmapLocalLane;

typedef struct _Dt_RECORD_RlRrCmrrObject
{
    uint8 CMRR_RR_OBJ_Track_ID;
    uint8 CMRR_RR_OBJ_Type;
    float32 CMRR_RR_OBJ_Width;
    float32 CMRR_RR_OBJ_Height;
    float32 CMRR_RR_OBJ_Length;
    float32 CMRR_RR_OBJ_HeadingAngle;
    float32 CMRR_RR_OBJ_Dy;
    float32 CMRR_RR_OBJ_Dx;
    float32 CMRR_RR_OBJ_Vy;
    float32 CMRR_RR_OBJ_Vx;
    float32 CMRR_RR_OBJ_XVelAbsolute;
    float32 CMRR_RR_OBJ_YVelAbsolute;
    float32 CMRR_RR_OBJ_Ay;
    float32 CMRR_RR_OBJ_Ax;
    uint8 CMRR_RR_OBJ_Location;
    float32 CMRR_RR_OBJ_ExistProb;
    uint8 CMRR_RR_Obj_DistLong_rms;
    uint8 CMRR_RR_Obj_VrelLong_rms;
    uint8 CMRR_RR_Obj_DistLat_rms;
    uint8 CMRR_RR_Obj_VrelLat_rms;
    float32 CMRR_RR_Obj_Orientation_rms;
    uint8 CMRR_RR_OBJ_state;
} Dt_RECORD_RlRrCmrrObject;

typedef Dt_RECORD_RlRrCmrrObject Dt_ARRAY_20_RlRrCmrrObject[20];

typedef struct _Dt_RECORD_RadarObstacleInfo
{
    float32 CMRR_FL_TiStamp;
    boolean CMRR_FL_TiStampStsGlbTiBas;
    boolean CMRR_FL_TiStampStsSyncToGatewy;
    boolean CMRR_FL_TiStampStsTiLeap;
    boolean CMRR_FL_TiStampStsTiOut;
    uint16 CMRR_FL_measure_Latency;
    float32 CMRR_FR_TiStamp;
    boolean CMRR_FR_TiStampStsGlbTiBas;
    boolean CMRR_FR_TiStampStsSyncToGatewy;
    boolean CMRR_FR_TiStampStsTiLeap;
    boolean CMRR_FR_TiStampStsTiOut;
    uint16 CMRR_FR_measure_Latency;
    float32 CMRR_RL_TiStamp;
    boolean CMRR_RL_TiStampStsGlbTiBas;
    boolean CMRR_RL_TiStampStsSyncToGatewy;
    boolean CMRR_RL_TiStampStsTiLeap;
    boolean CMRR_RL_TiStampStsTiOut;
    uint16 CMRR_RL_measure_Latency;
    float32 CMRR_RR_TiStamp;
    boolean CMRR_RR_TiStampStsGlbTiBas;
    boolean CMRR_RR_TiStampStsSyncToGatewy;
    boolean CMRR_RR_TiStampStsTiLeap;
    boolean CMRR_RR_TiStampStsTiOut;
    uint16 CMRR_RR_measure_Latency;
    uint32 Array_Length_Dummy_0;
    Dt_ARRAY_72_FlFrCmrrFreespace FlFrCmrrFreespace;
    uint32 Array_Length_Dummy_1;
    Dt_ARRAY_72_RlRrCmrrFreespace RlRrCmrrFreespace;
    uint32 Array_Length_Dummy_2;
    Dt_ARRAY_20_FlFrCmrrObject FlFrCmrrObject;
    uint32 Array_Length_Dummy_3;
    Dt_ARRAY_20_RlRrCmrrObject RlRrCmrrObject;
} Dt_RECORD_RadarObstacleInfo;

typedef struct _Dt_RECORD_RtkInfo
{
    float64 Acc_X;
    float64 Acc_Y;
    float64 Gyro_Z;
    float64 INS_Pitch;
    float64 INS_Roll;
    float64 INS_Heading;
    float64 INS_LocatHeight;
    uint64 INS_LatitudeLongitude_time_stamp;
    float64 INS_Latitude;
    float64 INS_Longitude;
    float64 INS_NorthSpd;
    float64 INS_EastSpd;
    uint64 GPS_LatitudeLongitude_time_stamp;
    float64 GPS_Latitude;
    float64 GPS_Longitude;
    uint64 GPS_HeightAndWeek_time_stamp;
    float64 GPS_LocatHeight;
} Dt_RECORD_RtkInfo;

typedef Dt_RECORD_SurViewParkingSpace Dt_ARRAY_16_SurViewParkingSpace[16];

typedef struct _Dt_RECORD_CameraParkInfos
{
    uint32 frame_index;
    uint64 time_stamp;
    uint32 parking_space_num;
    Dt_ARRAY_16_SurViewParkingSpace SurViewParkingSpace;
} Dt_RECORD_CameraParkInfos;

typedef Dt_RECORD_TrafficMarkings Dt_ARRAY_16_TrafficMarkings[16];

typedef struct _Dt_RECORD_TrafficSigns
{
    uint32 id;
    uint32 type;
    uint32 confidence;
    float64 pos_y;
    float64 pos_x;
    float64 pos_z;
} Dt_RECORD_TrafficSigns;

typedef Dt_RECORD_TrafficSigns Dt_ARRAY_16_TrafficSigns[16];

typedef struct _Dt_RECORD_EnvModelInfos
{
    uint32 frame_index;
    uint64 time_stamp;
    uint32 obstacle_num;
    uint32 traffic_sign_num;
    uint32 traffic_marking_num;
    uint32 traffic_light_num;
    uint32 resvered;
    Dt_RECORD_Lanes Lanes;
    Dt_ARRAY_64_Obstacles Obstacles;
    Dt_ARRAY_16_TrafficSigns TrafficSigns;
    Dt_ARRAY_16_TrafficMarkings TrafficMarkings;
    Dt_ARRAY_8_TrafficLights TrafficLights;
    Dt_ARRAY_80000_ObstacleGridMap ObstacleGridMap;
} Dt_RECORD_EnvModelInfos;

typedef struct _Dt_RECORD_FrontCameraInfos
{
    uint32 frame_index;
    uint64 time_stamp;
    uint32 obstacle_num;
    uint32 traffic_sign_num;
    uint32 traffic_marking_num;
    uint32 traffic_light_num;
    Dt_RECORD_Lanes Lanes;
    Dt_ARRAY_64_Obstacles Obstacles;
    Dt_ARRAY_16_TrafficSigns TrafficSigns;
    Dt_ARRAY_16_TrafficMarkings TrafficMarkings;
    Dt_ARRAY_8_TrafficLights TrafficLights;
    uint32 nSegDataWidth;
    uint32 nSegDataHeight;
    Dt_ARRAY_262144_szSegData szSegData;
    uint32 nImageWidth;
    uint32 nImageHeight;
    Dt_ARRAY_230400_szImageData szImageData;
} Dt_RECORD_FrontCameraInfos;

typedef struct _Dt_RECORD_ObstaclesAndTrafficInfos
{
    uint32 frame_index;
    uint64 time_stamp;
    uint32 obstacle_num;
    uint32 traffic_sign_num;
    uint32 traffic_marking_num;
    uint32 traffic_light_num;
    Dt_ARRAY_64_Obstacles Obstacles;
    Dt_RECORD_Lanes Lanes;
    Dt_ARRAY_16_TrafficSigns TrafficSigns;
    Dt_ARRAY_16_TrafficMarkings TrafficMarkings;
    Dt_ARRAY_8_TrafficLights TrafficLights;
    uint32 nSegDataWidth;
    uint32 nSegDataHeight;
    Dt_ARRAY_262144_szSegData szSegData;
    uint32 nImageWidth;
    uint32 nImageHeight;
    Dt_ARRAY_230400_szImageData szImageData;
} Dt_RECORD_ObstaclesAndTrafficInfos;

typedef Dt_RECORD_TrajectoryPoints Dt_ARRAY_500_TrajectoryPoints[500];

typedef struct _Dt_RECORD_TrajectoryPointsInfos
{
    float64 origin_yaw;
    uint32 point_num;
    uint32 Array_Length_Dummy_0;
    Dt_ARRAY_500_TrajectoryPoints TrajectoryPoints;
    uint32 decision;
    boolean hold;
    uint32 direction;
} Dt_RECORD_TrajectoryPointsInfos;

typedef Dt_RECORD_UsscParkingLeft Dt_ARRAY_6_UsscParkingLeft[6];

typedef struct _Dt_RECORD_UsscParkingRight
{
    uint8 USSC_Slotpropety_R;
    uint8 USSC_SlotConfidence_R;
    uint8 USSC_SlotConfidence_Flag_R;
    boolean USSC_SlotupdateOcd_R;
    sint16 USSC_SlotpiontAPosX_R;
    sint16 USSC_SlotpiontAPosY_R;
    sint16 USSC_SlotpiontBPosX_R;
    sint16 USSC_SlotpiontBPosY_R;
    sint16 USSC_SlotpiontCPosX_R;
    sint16 USSC_SlotpiontCPosY_R;
    sint16 USSC_SlotpiontDPosX_R;
    sint16 USSC_SlotpiontDPosY_R;
    uint8 USSC_PrkgSlotSta_R;
} Dt_RECORD_UsscParkingRight;

typedef Dt_RECORD_UsscParkingRight Dt_ARRAY_6_UsscParkingRight[6];

typedef struct _Dt_RECORD_UltrasonicParkInfos
{
    float32 APA_PrkgSlot_TimeStamp;
    boolean APA_PrkgSlot_TimeStampSts;
    uint8 USSC_slot_num;
    uint32 Array_Length_Dummy_0;
    Dt_ARRAY_6_UsscParkingLeft UsscParkingLeft;
    uint32 Array_Length_Dummy_1;
    Dt_ARRAY_6_UsscParkingRight UlsscParkingRight;
} Dt_RECORD_UltrasonicParkInfos;

typedef struct _Dt_RECORD_USSCTimeStamp
{
    float32 USSC_FLC_TimeStamp;
    float32 USSC_FLM_TimeStamp;
    float32 USSC_FLS_TimeStamp;
    float32 USSC_FRC_TimeStamp;
    float32 USSC_FRM_TimeStamp;
    float32 USSC_FRS_TimeStamp;
    float32 USSC_RLC_TimeStamp;
    float32 USSC_RLM_TimeStamp;
    float32 USSC_RLS_TimeStamp;
    float32 USSC_RRC_TimeStamp;
    float32 USSC_RRM_TimeStamp;
    float32 USSC_RRS_TimeStamp;
    boolean USSC_FLC_TimeStampSts;
    boolean USSC_FLM_TimeStampSts;
    boolean USSC_FLS_TimeStampSts;
    boolean USSC_FRC_TimeStampSts;
    boolean USSC_FRM_TimeStampSts;
    boolean USSC_FRS_TimeStampSts;
    boolean USSC_RLC_TimeStampSts;
    boolean USSC_RLM_TimeStampSts;
    boolean USSC_RLS_TimeStampSts;
    boolean USSC_RRC_TimeStampSts;
    boolean USSC_RRM_TimeStampSts;
    boolean USSC_RRS_TimeStampSts;
} Dt_RECORD_USSCTimeStamp;

typedef struct _Dt_RECORD_VehicleInfo
{
    uint64 PDCU_0x214_time_stamp;
    uint8 PDCU_GearSig;
    uint64 EPS_0x1C2_time_stamp;
    float32 EPS_SteerWhlAgSig;
    float32 EPS_SteerWhlRotSpd;
    uint8 EPS_SteerWhlRotSpdDir;
    uint64 ESP_0x125_time_stamp;
    uint8 ESP_WhlOdoEdges_FL;
    uint8 ESP_WhlOdoEdges_FR;
    uint8 ESP_WhlOdoEdges_RL;
    uint8 ESP_WhlOdoEdges_RR;
} Dt_RECORD_VehicleInfo;

typedef struct _Dt_RECORD_LocalizationInfo
{
    uint64 time_stamp;
    Dt_RECORD_LocalizationResult LocalizationResult;
    float64 Latitude;
    float64 Longitude;
    float64 yaw;
    float64 pitch;
    float64 roll;
    float64 yawrate;
    float64 a_lon;
    float64 a_lat;
    float64 v_lat;
    float64 v_lon;
} Dt_RECORD_LocalizationInfo;



#endif