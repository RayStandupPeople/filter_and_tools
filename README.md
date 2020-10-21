# filter_and_tools
---
1.简述：<br>
  该项目包含常用滤波算法及关联工具。<br>
    1.其中滤波算法目前支持中值滤波和均值滤波，用于对数据进行镇定优化；<br>
    2.关联工具包含log文件解析及可视化工具、数据优化分析工具及自动化bash脚本；<br>
    
---    
2.文件结构：<br>
  filter     <br>
  ->build            (临时文件及可执行文件)<br>
  ->SRC              (存放算法源文件)<br>
  ->LIB              (存放算法库文件)<br>
  ->LOG              (存放录制文件)<br>
  ->Tools            (存放工具文件)<br>
  ->CMakeLists.txt   (CMAKE脚本)<br>

--- 
3.功能描述：<br>
  3.1工具类：<br>
   × log文件解析及可视化工具（Tools/logfile_analysis_tool.py）<br>
    1.自动解析log文件： 基于属性关键字利用正则表达式对数据文件进行解析；<br>
    2.动态播放视觉障碍物目标的位置、ID、和类型属性，支持自定义位置播放；<br>
    3.支持特定目标的属性分析，支持基于ID进行特定目标的属性分析；<br>
    4.Log文件的概况显示；<br>
    <br>
  × 数据优化分析工具(Tools/dataDis.py)<br>
    1.读取数据对比文件，并进行可视化<br>
    <br>
  × 自动化bash脚本(Tools/start.bash)<br>
    1.自动执行数据优化算法程序及可视化程序<br>
    <br>
    <br>  
  3.2算法类：<br>
  × 滤波器（SRC/midian_filter.cc）<br>
    1.利用中值滤波（默认）或均值滤波对数据进行镇定优化处理<br>
    2.将原始及优化结果输出至数据对比文件<br>
    
---   
4.DEMO：<br>
× 修改 Tools/logfile_analysis_tool.py 中的 USER DASH BOARD 内容：<br>
![0](https://github.com/RayStandupPeople/filter_and_tools/raw/master/Pictures/userBash.png)<br>
× 运行 Tools/logfile_analysis_tool.py， 播放数据文件：<br>
![0](https://github.com/RayStandupPeople/filter_and_tools/raw/master/Pictures/log_analysis.png)<br>
× 运行 Tools/start.bash 执行数据滤波优化及对应数据对比可视化工具：<br>
![0](https://github.com/RayStandupPeople/filter_and_tools/raw/master/Pictures/Optimiz_res.png)<br>
  
    
