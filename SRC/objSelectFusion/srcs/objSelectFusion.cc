#include "../libs/objSelectFusion.h"

//@breaf: sync objlist selection and grid map selection. 
//		  strategy: set obj list interaction as default, and update it's properties(obj.s & position) when needed 
//@param grid_flg_lane   : the collision flag of grid map collision check for specified lane; 0-> no,  1->static, 2->dynamic
//@param grid_s_lane     : the distance s of grid map collision check for specified lane; unit meter
//@param selectObj_lane  : the setected obj struct of cipv selection for specified lane; Position: 0->none, 1->dynamic, 2->static
void decision::objSelectFusion(int grid_flg_lane, double grid_s_lane, objSec* selectObj_lane)
{
	DEBUG("zlm::objSelectFusion:grid_flg_lane =%d, grid_s_lane =%f, selectObj_lane.s =%f \r\n",grid_flg_lane,  grid_s_lane, selectObj_lane->obj.pos_s);
	if(selectObj_lane->postion == 0)  // obj List : no obj output
	{
		switch (grid_flg_lane)
		{
			case 0: // grid map: no obj
				break;
			case 1: // grid map: static obj
				selectObj_lane->postion = 2;
				selectObj_lane->obj.pos_s = grid_s_lane; 
				break;
			case 2: // grid map: dynamic obj
				selectObj_lane->postion = 1;
				selectObj_lane->obj.pos_s = grid_s_lane; 
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
				selectObj_lane->postion = grid_s_lane < selectObj_lane->obj.pos_s ? 2:1;
				selectObj_lane->obj.pos_s = min(grid_s_lane, selectObj_lane->obj.pos_s); 
				break;
			case 2: // grid map: dynamic obj
				selectObj_lane->postion = 1; 
				selectObj_lane->obj.pos_s = min(grid_s_lane, selectObj_lane->obj.pos_s); 
				break;
		}
	}
	DEBUG("zlm::objSelectFusion:selectObj_lane->postion=%d, selectObj_lane_fusion.s=%f, selectObj_lane_fusion.spdX=%f\r\n", \
	 selectObj_lane->postion, selectObj_lane->obj.pos_s,selectObj_lane->obj.abs_acc_x);
}