echo "load && display Tool"
set log_file_dir=log/log_list
set src_filr_dir=.
set src_filr_name=logfile_analysis_tool.py

protoc -I=libs --python_out=libs libs/types.proto

for %%i in (%log_file_dir%\*.*) do (
    @REM echo "%%i"
    set log_file_name=%%i
)

start python %src_filr_dir%\%src_filr_name% %log_file_name%

