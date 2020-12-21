#include <iostream>
#include <cmath>

#include <fstream>
#include <sstream>

#include "matplotlibcpp.h"
#include "Rte_Type.h"
#include "decision_struct.h"
#include "user_struct.h"

namespace plt = matplotlibcpp;

void plot_vehicleCoordi_wind(const hdMapTrajectory &Trajectory, const Dt_RECORD_EnvModelInfos &envModelInfo, const objSecList &selectObj);

void plot_globalCoordi_wind(const Dt_RECORD_LocalizationInfo &localInfos, const Dt_RECORD_HdmapInfo &hdmapInfos);
void plot_mapCoordi_wind(const Dt_RECORD_LocalizationInfo &localInfos, const Dt_RECORD_HdmapFrontPLane &globePLane);
void plot_infoList_wind();

int plot(const hdMapTrajectory &Trajectory, const Dt_RECORD_HdmapInfo &hdmapInfos, const Dt_RECORD_HdmapFrontPLane &globePLane, \
    const Dt_RECORD_LocalizationInfo &localInfos, const Dt_RECORD_EnvModelInfos &envModelInfo, const objSecList &selectObj);