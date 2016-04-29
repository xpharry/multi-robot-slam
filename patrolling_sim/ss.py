import sys, time, os, glob, shutil

MAP = 'cumberland'
NROBOTS = '6'
INITPOS = 'default'
ALG_SHORT = 'HCR'
LOC_MODE = 'odom'
NAV_MODULE = 'ros' # 'thin_navigation'
GWAIT = '3' # 0, 10
COMMDELAY = '0.2'
TERM = 'gnome-terminal'
TIMEOUT = 0
CUSTOM_STAGE = False
SPEEDUP = 1.0

roscore_cmd = 'gnome-terminal -e "bash -c \'roscore\'" &'

os.system(roscore_cmd)
os.system('sleep 3')
os.system('rosparam set /use_sim_time true')
os.system("rosparam set /goal_reached_wait "+GWAIT)
os.system("rosparam set /communication_delay "+str(COMMDELAY))
#    os.system("rosparam set /lost_message_rate "+LOSTMSGRATE)
os.system("rosparam set /navigation_module "+NAV_MODULE)
os.system("rosparam set /initial_positions "+INITPOS)

cmd_monitor = 'rosrun patrolling_sim monitor '+MAP+' '+ALG_SHORT+' '+NROBOTS  

cmd_stage = 'roslaunch patrolling_sim map.launch map:='+MAP

os.system('gnome-terminal --tab -e  "bash -c \''+cmd_monitor+'\'" --tab -e "bash -c \''+cmd_stage+'\'" &')

os.system('sleep 3')