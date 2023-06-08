#from Object import Object
import cv2
import numpy as np

colors ={'green':(np.array([60,127,127]) , np.array([90,255,255])),
'red':(np.array([0,127,127]) , np.array([30,255,255])),
'blue':(np.array([120,127,127]) , np.array([150,255,255])),
'yellow':(np.array([30,127,127]) , np.array([60,255,255])),
'white':(np.array([0,0,255]) , np.array([255,255,255])),
'black':(np.array([0,0,0]) , np.array([255,255,0])),
'gray':(np.array([0,0,0]) , np.array([0,0,255])),
'purple':(np.array([130,0,0]) , np.array([140,255,255])),
'brown':(np.array([10,127,127]) , np.array([20,255,255])),
'orange':(np.array([8,127,127]) , np.array([22,255,255]))}

def color_detection(frame):
    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    frame = frame.reshape((-1,3))	
    # convert to np.float32
    frame = np.float32(frame)

    # define criteria, number of clusters(K) and apply kmeans()
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    K = 2
    ret,label,center = cv2.kmeans(frame,K,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
    # Now convert back into uint8, and make original image
    center = np.uint8(center)
    res = center[label.flatten()]
    res2 = res.reshape((frame.shape))
    #cv.imshow('00',res2)
    #cv.waitKey(0)
    #Scv.destroyAllWindows()
    _,count = np.unique(label.flatten(),return_counts=True)
    center_idx = np.argmax(count)
    color_arr = center[center_idx]
    color = 'hhhhhh'
    for k,val in colors.items():
        h = color_arr[0]
        s = color_arr[1]
        v = color_arr[2]
        if (h < val[1][0]) and (h > val[0][0]) and (s < val[1][1]) and (s > val[0][1]) and (v < val[1][2]) and (v > val[0][2]):
            color = k
    return color
	
def alphabetic_detection():

    pass

def yolo():
    pass

def object_detection():
    pass
