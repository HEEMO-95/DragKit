from DragKit import *

action, start_point, stop_point = do_stop()

if a == 'stopped':
    a = go_back(start_point)

if a == 'got_back':
    do_scan()
