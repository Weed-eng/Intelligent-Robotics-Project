import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/zhoushiwang/human_nav_ws/install/human_aware_nav'
