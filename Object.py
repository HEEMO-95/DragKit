class Object():
    def __init__(self,color,shape,alphabet,xy):
        self.color = color
        self.shape = shape
        self.alphabet = alphabet
        self.xy = xy
    
    def __eq__(self,x):      
        return (self.color == x.color) and (self.shape == x.shape) and (self.alphabet == x.alphabet)
