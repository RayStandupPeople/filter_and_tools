echo "load && display Tool"
set log_file_dir=E:\vmShare\LOGFILE
set src_filr_dir=C:\Users\Administrator\Documents\BAICWorkSpace\filter\Tools
set src_filr_name=logfile_analysis_tool.py

for %%i in (%log_file_dir%\*.log) do (
    @REM echo "%%i"
    set log_file_name=%%i
)

python %src_filr_dir%\%src_filr_name% %log_file_name%