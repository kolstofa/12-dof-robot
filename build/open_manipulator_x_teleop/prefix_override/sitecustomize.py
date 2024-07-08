import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/x/Desktop/workspace/omx_tester/install/open_manipulator_x_teleop'
