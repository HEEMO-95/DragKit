<<<<<<< HEAD
class Object():
    def __init__(self,color,shape,alphabet,xy):
        self.color = color
        self.shape = shape
        self.alphabet = alphabet
        self.xy = xy
    
    def __eq__(self,x):      
        return (self.color == x.color) and (self.shape == x.shape) and (self.alphabet == x.alphabet)
=======
''' this is by all means not the way to do it
but its a step in the dircetion to reach a common ground
i wait for your feedback'''

visted_opjects = []

'''when the waypoint reached'''
start = True

'''start identfiying'''

opjects_list = calsses_in_frame()  # < from ai

for opject in opjects_list():

    if opject not in visted_opjects:

        opject_of_interst = opject
        a, start_location, stop_location = do_stop()

        '''do stop will mark the location in case of opject lost during breaking; we can do go_to(start_location)
        start_location is the location when the do_stop() action started'''


def get_data(opject_of_interst):
    try:
        x, y = opject_of_interst  # opject_of_interst center pixel coordiantes in the picture frame 
        return x,y
    except:
        '''rise lost flag to do go_to(start_location)'''
        a = go_to()



if (a == 'stopped' or 'got_back') and get_data:
    a = align(x,y)

    if a == 'aligned':

        a = winch(n)  # n is the pylon adress
        if a == 'done_winch':
            visted_opjects.abbend(opject_of_interst)



if len(visted_opjects) == 5:
    '''change mode to RTL (return to land) mission done
    otherwise FailsSafe will land in the case of mission wasnt done'''  
>>>>>>> 5aa49a1218e16317da8461881b73e046581c96dd
