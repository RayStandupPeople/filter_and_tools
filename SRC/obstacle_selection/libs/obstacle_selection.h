#pragma once
#include <iostream>
#include <vector>
#include <math.h>
#include "Rte_Type.h"
class decision{
public:
    double distance(double x1, double y1, double x2, double y2);

    int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

    int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

    std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, \
        const std::vector<double> &maps_y);


};

