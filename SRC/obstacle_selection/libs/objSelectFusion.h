#pragma once

#include <iostream>
#include <vector>
#include <math.h>
#include <cmath>
#include "Rte_Type.h"
#include "string.h"
#include "../../common/libs/user_struct.h"
#include "decision_struct.h"


class fusion_decision{
public:
  void objSelectFusion(int grid_flg_lane, double grid_s_lane, objSec* selectObj_lane, string lane_name);

};

