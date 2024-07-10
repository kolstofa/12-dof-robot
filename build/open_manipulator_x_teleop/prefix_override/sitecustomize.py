import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/x/Desktop/workspace/12-dof-robot/install/open_manipulator_x_teleop'
