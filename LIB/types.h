#pragma once

struct obj_sel{
    int id;
    int type;
    double pos_x;
    double pos_y;
    double heading;
    double rel_speed_x;
    double rel_speed_y;
};

struct objSec
{
    int position;
    obj_sel obj;
};

struct objSecList
{
    objSec frontMid;
    objSec frontMidLeft;
    objSec frontMidRight;
    objSec objSecAEB;
};


struct objFrame
{
    int frame_num;
    std::vector<obj_sel> obj;
};
