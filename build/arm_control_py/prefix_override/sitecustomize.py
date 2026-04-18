import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hamza/Logistics-Robot-with-Computer-Vision/install/arm_control_py'
