protoc -I=libs --python_out=libs libs/types.proto

logfile_dir=log/log_list
py_src_dir=.
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

