# filter_and_tools
1.简述：
  该项目包含常用滤波算法及关联工具。
    1.其中滤波算法目前支持中值滤波和均值滤波，用于对数据进行镇定优化；
    2.关联工具包含log文件解析及可视化工具、数据优化分析工具及自动化bash脚本；
    
    
2.文件结构：
  filter     
  ->build            (临时文件及可执行文件)
  ->SRC              (存放算法源文件)
  ->LIB              (存放算法库文件)
  ->LOG              (存放录制文件)
  ->Tools
  ->CMakeLists.txt   (CMAKE脚本)
  
  
3.功能描述：
  3.1工具类：
  × log文件解析及可视化工具（Tools/logfile_analysis_tool.py）
    1.自动解析log文件： 基于属性关键字利用正则表达式对数据文件进行解析；
    2.动态播放视觉障碍物目标的位置、ID、和类型属性，支持自定义位置播放；
    3.支持特定目标的属性分析，支持基于ID进行特定目标的属性分析；
    4.Log文件的概况显示；
  × 数据优化分析工具(Tools/dataDis.py)
    1.读取数据对比文件，并进行可视化
  × 自动化bash脚本(Tools/start.bash)
    1.自动执行数据优化算法程序及可视化程序
    
  3.2算法类：
  × 滤波器（SRC/midian_filter.cc）
    1.利用中值滤波（默认）或均值滤波对数据进行镇定优化处理
    2.将原始及优化结果输出至数据对比文件
   
4. DEMO：
  × 修改 Tools/logfile_analysis_tool.py 中的 USER DASH BOARD 内容：
    
  × 运行 Tools/logfile_analysis_tool.py， 播放数据文件：
  
  × 运行 Tools/start.bash 执行数据滤波优化及对应数据对比可视化工具：
  
    
