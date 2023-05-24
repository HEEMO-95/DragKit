from DragKit import *

a, stop_point = do_stop()
if a == 'stopped':
    go_back(stop_point,yaw=1)
