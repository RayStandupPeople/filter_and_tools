#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>


// #include "../common/libs/functions.h"
#include "../common/libs/user_struct.h"
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
    objSel.frontMid.obj.s=21.5;      // 21.5
    objSel.frontMid.obj.d=1.5;      // 21.5

    objSel.frontMid.obj.abs_acc_x=4.3;  // 4.3

    coll_obj.obj_mid_flag =2;         // 0  1  
    coll_obj.s_m =17.9;                  // 0  18.9
    coll_obj.obj_lef_flag =2;
    coll_obj.s_l =3.5;
    coll_obj.obj_rig_flag=0;
    coll_obj.s_r =0;

    
    
    decision_obj.objSelectFusion(coll_obj.obj_mid_flag, coll_obj.s_m, &objSel.frontMid, "frontMid");
    decision_obj.objSelectFusion(coll_obj.obj_lef_flag, coll_obj.s_l, &objSel.frontLeft, "frontLeft");
    decision_obj.objSelectFusion(coll_obj.obj_rig_flag, coll_obj.s_r, &objSel.frontRight, "frontRight");

    return 0;
}