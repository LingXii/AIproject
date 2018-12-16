from square import square
import cv2
class board(object):

    start = 0
    goal = 0
    length = 20
    height = 20
    squares = []

    def __init__(self,img,length,height):
        """ Loads the board, creates a square for each point on the board, and links those points together"""
        self.length = int(length)
        self.height = int(height)
        self.squares = [[square(x,y) for x in range(self.length)] for y in range(self.height)] 
        for x in range(self.length):
            for y in range(self.height):
                self.squares[x][y] = square(x,y)

        for y in range(self.height):
            for x in range(self.length):
                self.squares[x][y].state = str(img[y][x])
                self.link(self.squares[x][y])
        self.start = self.squares[0][0]
        self.goal = self.squares[self.length - 1][self.height - 1]
    '''
            with open(path) as input_data:
                x = 0
                y = 0
                for y in range(self.height):
                    line = input_data.readline().strip()
                    for x in range(self.length):
                        self.squares[x][y].state = line[x]
                        self.link(self.squares[x][y])
                    #y = y + 1


                self.start = self.squares[0][0]
                self.goal = self.squares[self.length-1][self.height-1]
    '''

    def __str__(self):
        """ Returns an ASCII printout of the board and the current state of each square"""
        toreturn = ''
        for y in range(self.height):
            for x in range(self.length):
                 toreturn += str(self.squares[x][y])
            toreturn += "\n"
        return toreturn

    def lookup(self,x,y):
        """ Look to see if there is a valid square that we can travel to at x,y"""
        if (x < 0) or (y < 0) or (x > (self.length - 1)) or (y > (self.height - 1)) or self.squares[x][y].state == '255':
            return 0
        else:
            return self.squares[x][y]

    def link(self,square):
        """ Create links where available for the 8 main compass points between each square """
        x = square.x
        y = square.y

        if self.squares[x][y].state == '255':
            return
        square.northeast = self.lookup(x + 1, y - 1)
        square.northwest = self.lookup(x - 1, y - 1)
        square.southeast = self.lookup(x + 1, y + 1)
        square.southwest = self.lookup(x - 1, y + 1)
        square.north = self.lookup(x,y-1)
        square.west = self.lookup(x-1,y)
        square.south = self.lookup(x,y+1)
        square.east = self.lookup(x+1,y)
        
