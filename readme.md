Version 3.3  (2020,12,04)

update 3.3:
	* fix bug of display Target Object ID miss frames 
	
update 3.2:
	* add dicision_command display feature

update 3.1:
	* add keyboard input thread which help to skip frames 

usage£º
1. put the log file which need to be analysied into dir "log/log_list";
2. ShortCut: double click the shortcut, which will auto run the tool, which can help to do this:
	* init everything this tool need
	* parse logfile and display key datas
	* output data parsed as protobuf format file 
tips:
	* if you need specialized config, you can open the main python file, and search key word "xhw"...


feature: 
1. improve parsing ability, help to make sure geting the correct data 
2. enrich display content, detail as global path, localizaion....