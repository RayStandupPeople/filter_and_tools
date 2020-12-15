#pragma once

#include <iostream>
#include <vector>
#include <math.h>
#include "Rte_Type.h"
#include "string.h"
#include "../../common/libs/types.h"
#include "decision_struct.h"


class decision{
public:
  void objSelectFusion(int grid_flg_lane, double grid_s_lane, objSec* selectObj_lane);

};

