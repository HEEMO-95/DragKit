#from DragKit import *

#a, start_point, stop_point = do_stop()

#if a == 'stopped':
#    a = go_back(start_point)

#if a == 'got_back':
#    do_scan()
import subprocess
import os
import cv2
from AI import *
#os.system("sudo modprobe v4l2loopback exclusive_caps=1 max_buffer=2")
#print('detected cameras')
#e = subprocess.Popen(["gphoto2","--auto-detect"])
#print('start the camera')
#process = os.popen("gphoto2 --stdout --capture-movie | ffmpeg -i - -vcodec rawvideo -pix_fmt yuv420p -threads 0 -f v4l2 /dev/video0")

vid = cv2.VideoCapture('/dev/video0')
print('start capturing')
while(True):
    try: 
        # Capture the video frame
        # by frame
        ret, frame = vid.read()
        c,a = color_detection(frame)
        print(frame.shape)
        print(a)
        print(c)
        #Display the resulting frame
        cv2.imshow('frame', frame)
    except:
        continue
    # the 'q' button is set as the
    # quitting button you may use any
    # desired button of your choice
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
  	
# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()
