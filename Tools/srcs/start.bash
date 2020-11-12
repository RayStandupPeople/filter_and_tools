protoc -I=libs --python_out=libs libs/types.proto
python logfile_analysis_tool.py
