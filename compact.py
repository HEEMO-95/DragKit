from DragKit import *

a, stop_point = do_stop()

if a == 'stopped':
    a = go_back(stop_point)

if a == 'got_back':
    do_scan()
