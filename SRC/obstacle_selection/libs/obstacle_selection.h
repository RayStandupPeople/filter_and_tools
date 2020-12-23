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

    double distance(double x1, double y1, double x2, double y2);

    int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

    int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

    std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, \
        const std::vector<double> &maps_y);

    void get_refpath(const int &onpath, hdMapTrajectory *Trajectory, laneInfo *refpath);
    
    void convert_flat_to_vehicle(Dt_RECORD_LocalizationInfo *loc_info, laneInfo *refpath);

    void ObjDetect(int onpath, hdMapTrajectory *Trajectory, Dt_RECORD_HdmapInfo *hdmapInfos,\
        Dt_RECORD_HdmapFrontPLane *globePLane, Dt_RECORD_HdmapLocalLane *localPLanne, \
        Dt_RECORD_LocalizationInfo *localInfos, Dt_RECORD_EnvModelInfos *envModelInfo, EgoConfigPara ego_config, objSecList *selectObj);

};

