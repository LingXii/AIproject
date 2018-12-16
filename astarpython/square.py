class square(object):
    x = 0
    y = 0

    state = '9'

    north = 0
    northwest = 0
    west = 0
    southwest = 0
    south = 0
    southeast = 0
    east = 0
    northeast = 0
    
    def __init__(self,x,y):
        self.x = x
        self.y = y
    
    def __str__(self):
        return str(self.state)

    def __eq__(self, other):
        if isinstance(other, square):
            return ((self.y == other.y) and (self.x == other.x))
        else:
            return False

    def __ne__(self, other):
        return (not self.__eq__(other))

    def __hash__(self):
        return hash(str(self.x) + "," + str(self.y))
        

