#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>


#include "../common/libs/functions.h"

int main(int argc, const char** argv) {
    std::ifstream in_file("../../../log/geely.csv",std::ios::in);
    if(!in_file.is_open()){
        std::cout << "ERROR: OPEN  file "<<std::endl;
    }
    std::string line;
    double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
    std::vector<double> x_array;
    std::vector<double> y_array;
    std::vector<float>  s_array;

    while(!in_file.eof()){
        getline(in_file, line);
        std::istringstream iss(line);
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        x_array.push_back(x);
        y_array.push_back(y);
        s_array.push_back(s);
    }
    vector<double> sd_(2);
    sd_ = getFrenet(180.4,1992.9,3.14/2,x_array,y_array);
    std:: cout << sd_[0] << " " << sd_[1] << std::endl;
    return 0;
}
