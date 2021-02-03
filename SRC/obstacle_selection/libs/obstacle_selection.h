#pragma once
#include <iostream>
#include <vector>
#include <math.h>
#include "string.h"
#include "../../common/libs/Rte_Type.h"
#include "../../common/libs/user_struct.h"
#include "../../common/libs/decision_struct.h"


class decision{
public:
    void DealWithHdmap(Dt_RECORD_HdmapInfo *hdmapInfos, double xVeh, double yVeh, double heading, Dt_RECORD_HdmapFrontPLane *globePLane, Dt_RECORD_HdmapLocalLane *localPLanne, hdMapTrajectory *Trajectory);
    void DealWithNode(double dis, Dt_RECORD_HdMapLane *frontLane, laneInfo *globalTraj, double xVeh, double yVeh, double heading, double mapHeading);
    int generateTrajWithLineCurvature(double distance, double xs, double ys, double cs, double hs, double rs,double xt, double yt, double ct, double ht,double rt, laneInfo *frontTraj, double *sengmentDis);
    static int getGridCoordiFromParkXY(const double veh_heading, const double park_y_heading, const double x_veh, const double y_veh, const double x_pt, const double y_pt, double* x_out, double* y_out);
    double Distance(double x1, double y1, double x2, double y2);
    int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
    int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
    std::vector<double> GetFrenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
    void GetRefpath(const int &onpath, hdMapTrajectory *Trajectory, laneInfo *refpath, Dt_RECORD_LocalizationInfo *localizationInfo, decision_info *decisionInfo);
   	void GetSegsBoundary(int onpath, hdMapTrajectory* Trajectory, Dt_RECORD_HdmapFrontPLane* globePLane, Dt_RECORD_HdmapLocalLane* localPLanne, Dt_RECORD_LocalizationInfo *localizationInfo, const std::vector<double> &refpath_x, const std::vector<double> &refpath_y, std::vector<std::vector<double>> &segs_boundary);
    void AssignObstacleProperty(Obj_sel &left, const Dt_RECORD_Obstacles & right);
    void DebugObstacleProperty(const string &str, const objSec &obstacle);
    void ObjDetect(int onpath, hdMapTrajectory *Trajectory, Dt_RECORD_HdmapInfo *hdmapInfos,\
     Dt_RECORD_HdmapFrontPLane *globePLane, Dt_RECORD_HdmapLocalLane *localPLanne, \
     Dt_RECORD_LocalizationInfo *localInfos, Dt_RECORD_EnvModelInfos *envModelInfo, EgoConfigPara ego_config, decision_info *decisionInfo, objSecList *selectObj);
};

