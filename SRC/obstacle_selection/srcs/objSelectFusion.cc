#include "../libs/objSelectFusion.h"

//@breaf: V2.0 sync objlist selection and grid map selection. 
//		  strategy: 1. either obj or grid valid, choose it's property
//                  2. both obj and grid valid, when longitudinal distance err less than Threshold, the choose grid
// 					3. both...valid, when... more than Threshold, choose near one  
//@param grid_flg_lane               : the collision flag of grid map collision check for specified lane; 0-> no,  1->static, 2->dynamic
//@param grid_s_lane                 : the distance s of grid map collision check for specified lane; unit meter
//@param && output   selectObj_lane  : the struct of cipv selection for specified lane; 
//@param lane_name                   : indicate your interested lane
void fusion_decision::objSelectFusion(int grid_flg_lane, double grid_s_lane, objSec* selectObj_lane, string lane_name)
{
	double Threshold = 1; // unit: meter, help to distinguish whether two valid obs is the same one;
	DEBUG("OBJ SELECT FUSION----> %s [Before Fusion] : grid_flg, grid_s, selectObj_s =  %d   %2f   %2f\r\n",lane_name.c_str(), grid_flg_lane,  grid_s_lane, selectObj_lane->obj.s);
	if(selectObj_lane->postion == 0)  // obj List : no objList output
	{
		switch (grid_flg_lane)
		{
			case 0: // grid map: no obj
				selectObj_lane->obj.type = 255; // default invalid type
				selectObj_lane->obj.d = 0; // default invalid d
				break;
			case 1: // grid map: static obj
				selectObj_lane->postion = 2;
				selectObj_lane->obj.s = grid_s_lane; 
				selectObj_lane->obj.type = 255; // default invalid type
				selectObj_lane->obj.d = 0; // default invalid d
				break;
			case 2: // grid map: dynamic obj
				selectObj_lane->postion = 1;
				selectObj_lane->obj.s = grid_s_lane; 
				selectObj_lane->obj.type = 255; // default invalid type
				selectObj_lane->obj.d = 0; // default invalid d
				break;
		}

	}
	else   // obj List: dynamic obj Output
	{
		switch (grid_flg_lane)
		{
			case 0: // grid map: no obj
					break;
			case 1: // grid map: static obj
				if(std::abs(grid_s_lane - selectObj_lane->obj.s) < Threshold) // same obstalce
					break; // take obj_list as cipv
				else
				{
					if(grid_s_lane < selectObj_lane->obj.s) // grid is cipv, update property
					{
						selectObj_lane->postion = 2;
						selectObj_lane->obj.s = std::min(grid_s_lane, selectObj_lane->obj.s); 
						selectObj_lane->obj.type = 255; // default invalid type
						selectObj_lane->obj.d = 0; // default invalid d
					}
					break;	
				}
				
			case 2: // grid map: dynamic obj
				if(abs(grid_s_lane - selectObj_lane->obj.s) < Threshold) // same obstalce
					break; // take obj_list as cipv
				else
				{
					if(grid_s_lane < selectObj_lane->obj.s) // grid is cipv, update property
					{
						selectObj_lane->postion = 1; 
						selectObj_lane->obj.s = std::min(grid_s_lane, selectObj_lane->obj.s); 
						selectObj_lane->obj.type = 255; // default invalid type
						selectObj_lane->obj.d = 0; // default invalid d
					}
					break;
				}
		}
	}
	DEBUG("OBJ SELECT FUSION----> %s [After  Fusion] : selectObj_lane->postion, selectObj_lane_fusion.s,  selectObj_lane_fusion.d, selectObj_lane_fusion.type = %d   %2f   %2f   %d\r\n", \
	 lane_name.c_str(), selectObj_lane->postion, selectObj_lane->obj.s,  selectObj_lane->obj.d, selectObj_lane->obj.type);
}