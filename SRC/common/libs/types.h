#pragma once
#include<vector>

struct obj_sel{
    int id;
    int type;
    double pos_x;
    double pos_y;
    double heading;
    double rel_speed_x;
    double rel_speed_y;
    double pos_s;
    double pos_d;
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

struct Node
{
    double flat_X;
    double flat_Y;
    double Heading;
    double car_x;
    double car_y;
    double s;
    double d;

};

struct Path
{
    int id;
    std::vector<Node> data;
};

struct Location
{
    double flat_X;
    double flat_Y;
    double Heading;
    double car_x;
    double car_y;
    double s;
    double d;
};


