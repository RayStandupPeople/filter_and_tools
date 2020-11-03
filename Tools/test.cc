#include <fstream>
#include <sstream>
#include "../LIB/types.pb.h"
 int main(int argc, const char** argv) {
    pb_types::LogFile logfile_ptr;

    std::ifstream in_file;
    std::stringstream buffer;

    in_file.open("../log/obstacle_list_info_pb",std::ios::in);
    buffer << in_file.rdbuf();
    std::string str(buffer.str());
    std::cout << buffer.str();
    if(!logfile_ptr.ParseFromString(str)){
        std::cout << "ERROR with parse from file" << std::endl;
    }
    std::cout << logfile_ptr.frame_num();
    // for(int i=0;i<str.size();++i)
    // {
    //     std::cout
    // }
    
    return 0;
}