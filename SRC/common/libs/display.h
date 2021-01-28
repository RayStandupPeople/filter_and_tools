#include <iostream>
#include <cmath>

#include <fstream>
#include <sstream>

#include "matplotlibcpp.h"
#include "Rte_Type.h"
#include "decision_struct.h"
#include "user_struct.h"
#include "box.h"

enum class ObsType {Ego_Vehicle, CIPV_V1, CIPV_V2, NotImpor_Obs};
class VehicleSize{
public:
    double wheel_base = 3.75;
    double length = 4.68;
    double width = 1.93;
};
namespace plt = matplotlibcpp;
void plot_box(std::vector<double> cent_x, std::vector<double> cent_y, std::vector<double>heading, std::vector<int> type, const enum ObsType &obs_type, PlotItems &plot_items);
void plot_vehicleCoordi_wind(const hdMapTrajectory &hdMapTrajectory,const DecisionToPC &rev_DecisionToPC_data, const objSecList &selectObj, PlotItems &plot_items);

void plot_globalCoordi_wind(const Dt_RECORD_LocalizationInfo &localInfos, const Dt_RECORD_HdmapInfo &hdmapInfos, PlotItems  &plot_items);
void plot_mapCoordi_wind(const DecisionToPC &rev_DecisionToPC_data, PlotItems  &plot_items);
void plot_infoList_wind(const DecisionToPC &rev_DecisionToPC_data);

int plot(const DecisionToPC &rev_DecisionToPC_data, const hdMapTrajectory &hdMapTrajectory, const objSecList &selectObj, PlotItems  &plot_items);

// int plot(const Dt_RECORD_TrajectoryPointsInfos &trajectoryPointsInfo, const Dt_RECORD_HdmapInfo &hdmapInfos, const Dt_RECORD_HdmapFrontPLane &globePLane,\
// const Dt_RECORD_LocalizationInfo &localInfos, const Dt_RECORD_EnvModelInfos &envModelInfo, const Dt_RECORD_AccInfo &vehicleInfo, const objSecList &selectObj);

void plot_lane(const Dt_RECORD_TrajectoryPointsInfos &TrajectoryPoints, const DecisionToPC &rev_DecisionToPC_data, PlotItems &plot_items);
void plot_gridmap(const Dt_RECORD_EnvModelInfos &envModelInfo, PlotItems &plot_items);