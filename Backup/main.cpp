#include <iostream>
#include <cmath>
#include <vector>

#include <fstream>
#include <sstream>
#include "time.h"
#include "../libs/display.h"
namespace plt = matplotlibcpp;
class Plot{
    std::vector<double> x;
    std::vector<double> y;
    string s;
public:
    plt::Plot p;
    Plot() = default;
    Plot(const std::vector<double> &x_, const std::vector<double> &y_, const string &s_){
        x=x_;
        y=y_;
        s=s_;
        plt::Plot p("name1",x_,y_,s_);
    }
};

int main(){
    clock_t g_t_s, g_t_e;

    
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;

    for(int i=0;i<100;++i)
    {
        x.push_back(i);
        y.push_back(i);

        z.push_back(-i);
    }

  
    plt::ion();
    g_t_s = clock();
    // for(int i=0;i<100;++i)
    // {
    //     plt::plot(x, y,"ro");
    //     plt::pause(0.1);
    // }
    std::vector<double> a(1);
    std::vector<double> b(1);
    // plt::Plot x__("name",a,b,"ro");
    plt::Plot x__("name","ro");

    // plt::plot(x,z);
    // Plot p_(a,b,"ro");
    for(int i=0;i<100;++i)
    {
        x__.update(x,y);
    plt::pause(1);
    }
    
    // plt::pause(0.1);
    // for(int i=0;i<100;++i)
    // {
    //     p_.p.update(x,y);
    //     plt::pause(0.1);
    // }

     g_t_e = clock();
        std::cout << "----> PER frame time cost: " << (double)(g_t_e -g_t_s)/CLOCKS_PER_SEC << "s"<< std::endl;;

}