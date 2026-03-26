import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/a81r/Documents/MARCH ROVER/Robot_er_Haat-main(1)/Robot_er_Haat-main/install/rover_arm'
