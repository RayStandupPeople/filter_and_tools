#!/bin/bash 

logfile_dir=/mnt/hgfs/vmShare/LOGFILE
py_src_dir=/home/ze/Documents/LogAnalysis_Algos/Tools/srcs
py_src_name=logfile_analysis_tool.py

# get logfile name
for t in ${logfile_dir}/*.log;
do
    # echo ${t}
    logfile_name=${t}
done
# echo ${file_name}

cd ${py_src_dir}
python ${py_src_name} ${logfile_name}