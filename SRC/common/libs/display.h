#include <iostream>
#include <cmath>

#include <fstream>
#include <sstream>

#include "matplotlibcpp.h"
#include "Rte_Type.h"
#include "decision_struct.h"
#include "user_struct.h"
#include "box.h"

namespace plt = matplotlibcpp;
void plot_box(std::vector<double> cent_x, std::vector<double> cent_y, std::vector<double>heading);

void plot_vehicleCoordi_wind(const Dt_RECORD_TrajectoryPointsInfos &trajectoryPointsInfo, const Dt_RECORD_EnvModelInfos &envModelInfo, const objSecList &selectObj);

void plot_globalCoordi_wind(const Dt_RECORD_LocalizationInfo &localInfos, const Dt_RECORD_HdmapInfo &hdmapInfos);
void plot_mapCoordi_wind(const Dt_RECORD_LocalizationInfo &localInfos, const Dt_RECORD_HdmapFrontPLane &globePLane);
void plot_infoList_wind(const Dt_RECORD_HdmapInfo &hdmapInfos, const Dt_RECORD_AccInfo &vehicleInfo);

int plot(const Dt_RECORD_TrajectoryPointsInfos &trajectoryPointsInfo, const Dt_RECORD_HdmapInfo &hdmapInfos, const Dt_RECORD_HdmapFrontPLane &globePLane,\
const Dt_RECORD_LocalizationInfo &localInfos, const Dt_RECORD_EnvModelInfos &envModelInfo, const Dt_RECORD_AccInfo &vehicleInfo, const objSecList &selectObj);
