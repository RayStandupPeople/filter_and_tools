#include <fstream>
#include <sstream>
#include "../LIB/types.pb.h"
 int main(int argc, const char** argv) {
    pb_types::LogFile logfile_ptr;

    std::ifstream in_file;
    std::stringstream buffer;
    // std::string str;
    in_file.open("../log/obstacle_list_info_pb",std::ios::in);
    // getline(in_file, str);
    // std::cout << str;
    buffer << in_file.rdbuf();
    std::string str(buffer.str());
    // std::cout << buffer.str();
    if(!logfile_ptr.ParseFromString(str)){
        std::cout << "ERROR with parse from file" << std::endl;
    }
    // std::cout << logfile_ptr.frame_num() << std::endl;
    // std::cout << logfile_ptr.frame_size()<< std::endl;
   
    for(int i =0;i<logfile_ptr.frame_size();++i)
    {   pb_types::Frame frame_ = logfile_ptr.frame(i);
        std::cout << "frame_num: " << frame_.id() << std::endl;
        for(int j=0;j<frame_.obstacle_size();++j)
        {
            pb_types::Obstacle obstacle_ = frame_.obstacle(j);
            std::cout << obstacle_.id() << std::endl;
            std::cout << obstacle_.type() << std::endl;
            std::cout << obstacle_.pos_x() << std::endl;
            std::cout << obstacle_.pos_y() << std::endl;
        }
        std::cout << std::endl;
    }
    
    // for(int i=0;i<str.size();++i)
    // {
    //     std::cout
    // }
    
    return 0;
}