#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>


#include "../common/libs/functions.h"
#include "../common/libs/types.h"
#include "libs/objSelectFusion.h"

int main(int argc, const char** argv) {

    decision decision_obj;
    gridmap_coll_obj coll_obj;
    objSecList objSel;
    memset(&objSel,0,sizeof(objSecList));
    memset(&coll_obj,0,sizeof(gridmap_coll_obj));

    //use case
    objSel.frontMid.postion=1;        // 1 
    objSel.frontMid.obj.id=6;         // 6
    objSel.frontMid.obj.pos_s=21.5;      // 21.5
    objSel.frontMid.obj.abs_acc_x=4.3;  // 4.3

    coll_obj.obj_mid_flag =1;         // 0  1  
    coll_obj.s_m =28.9;                  // 0  18.9
    coll_obj.obj_lef_flag =2;
    coll_obj.s_l =3.5;
    coll_obj.obj_rig_flag=0;
    coll_obj.s_r =0;

    
    
    decision_obj.objSelectFusion(coll_obj.obj_mid_flag, coll_obj.s_m, &objSel.frontMid);
    decision_obj.objSelectFusion(coll_obj.obj_lef_flag, coll_obj.s_l, &objSel.frontLeft);
    decision_obj.objSelectFusion(coll_obj.obj_rig_flag, coll_obj.s_r, &objSel.frontRight);

    return 0;
}