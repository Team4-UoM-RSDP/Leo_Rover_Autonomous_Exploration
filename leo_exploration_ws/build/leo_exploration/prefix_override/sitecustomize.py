import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/group4/LeoRoverAutonomousExploration/leo_exploration_ws/install/leo_exploration'
