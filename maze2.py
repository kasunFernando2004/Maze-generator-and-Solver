# Code written by Kasun Fernando
# Code Purpose: Implementation of iterative depth first search maze generation along with A* algorithm for solving the maze. 
import pygame
import sys
import  random
from hroitij import Stack
import time 
pygame.init()
import numpy as np
# initialising size of a path pixel, and color
size =5
white = (255,255,255)
red = (255,0,0)
blue = (0,0,255)
green = (0,255,0)
black = (0,0,0)
#function to convert from (x,y) position on screen to (i,j) index in Maze Array
def getX(x):
    return int((1/size)*(x-(50+size)))
def getY(y):
    return int((1/size)*(y-(50+size)))
def invGetX(x):
    return int(size*x+(50+size))
def invGetY(y):
    return int(size*y+(50+size))

#class to represent an individual square
class Cell():
    def __init__(self,x,y,color): # has a position and color
        self.y = y
        self.x = x
        self.color = color

    def drawCell(self): # methodd to draw the cell
        for i in range(size):
            for j in range(size):
                screen.set_at((self.x+i,self.y+j),self.color)
class Maze(): #class for the whole maze

    def __init__(self,xMazeSize,yMazeSize): #init takes in the size of the maze
        #x2 size because a maze with s visitable squares in a row will need 2*s+1 walls
        self.xMazeSize = 2*xMazeSize+1
        self.yMazeSize = 2*yMazeSize+1
        # actual maze dimensions
        self.screen_width = 2*size*xMazeSize+100
        self.screen_height = 2*size*yMazeSize+100
        # creats a 2d array containing each visitable cell
        Mazze = [[None]*self.xMazeSize for _ in range(self.yMazeSize)]
        for i in range(self.xMazeSize):
            for j in range(self.yMazeSize):
                if ((i+1)%2==0):
                    Mazze[j][i] = Cell(50+size*i,50+size*j,black)
                else:
                    Mazze[j][i] = Cell(50+size*i,50+size*j,black)
                if ((j+1)%2!=0):
                    Mazze[j][i] = Cell(50+size*i,50+size*j,black)
                Mazze[j][i].visited = 0
        self.Mazze = Mazze
        #empty list that will contain the cells we want to redraw between frames
        self.cellsToReDraw = []
    def drawMazeCells(self): #draws all the maze cells
        Mazze = self.Mazze
        for i in range(self.xMazeSize):
            for j in range(self.yMazeSize):
                Mazze[j][i].drawCell()
    def flipVisited(self,position):   #tells the computer the cell has been visited, needed for maze generation and solving
        x = position[0]
        y = position[1]
        self.Mazze[y][x].visited= 1
    def redrawCells(self): # takes in the cells we need to redraw and redraws them
        cells = self.cellsToReDraw
        for cell in cells:
            self.Mazze[cell[0]][cell[1]].drawCell()
        self.cellsToReDraw=[]

#For the implementation of the A* algorithm
# it is a graph traversal algorithm, this class is designed to create a simplified version of the maze
# where a node is placed at all non corridors in the maze (corridor is "====", a portion of the maze where you can only go forward-backward,up-down)
class Node(): 
    nodeMazze = []
    gguh = 0
    #the below two Matrices are there so that when we iterate through the maze, we can tell whether a node
    # has neighbours (indicate by a one), or doesn't (indicate by a one). 1 through 13 represent the interesting points in a maze
    # for example a left L cannot pass into a right L thus there is a 0 there
    lookingDownMatrix = [
        [0,1,1,1,0,0,1,1,0,1,0,0,1],
        [0,1,1,1,0,0,1,1,0,1,0,0,1],
        [0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,1,1,1,0,0,1,1,0,1,0,0,1],
        [0,1,1,1,0,0,1,1,0,1,0,0,1],
        [0,1,1,1,0,0,1,1,0,1,0,0,1],
        [0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,1,1,1,0,0,1,1,0,1,0,0,1],
        [0,1,1,1,0,0,1,1,0,1,0,0,1],
    ] # y is looking at x, x is looking at y for up
    lookingLeftMatrix = [
        [1,1,1,0,0,1,1,0,1,0,0,0,1],
        [0,0,0,0,0,0,0,0,0,0,0,0,0],
        [1,1,1,0,0,1,1,0,1,0,0,0,1],
        [1,1,1,0,0,1,1,0,1,0,0,0,1],
        [1,1,1,0,0,1,1,0,1,0,0,0,1],
        [0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0],
        [1,1,1,0,0,1,1,0,1,0,0,0,1],
        [0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0],
        [1,1,1,0,0,1,1,0,1,0,0,0,1],
        [0,0,0,0,0,0,0,0,0,0,0,0,0],
        [1,1,1,0,0,1,1,0,1,0,0,0,1]
    ]  #y is looking at x, x is looking at y for right
    def __init__(self,grid,x,y,NodeType): 
        self.x=x
        self.y=y
        self.nodeType = NodeType
        Node.gguh=1
        self.otherNodes = [None,None,None,None] #left top right bottom, array for a nodes neighbours, will pass in address of those node objects
        # top left of the maze
        endX = 1
        endY = 1
        self.heuristic = abs(endX-self.x)+abs(endY-self.y)  # represents manhatten distance from node to finish line, needed for A*
        Node.nodeMazze.append(self)
    def addOtherNode(self,otherNode,otherNodePosRelative): #allows us to add a nodes neighbours
        if otherNodePosRelative==0:
            self.otherNodes[0] = [otherNode,abs(otherNode.x-self.x)]
        if otherNodePosRelative==1:
            self.otherNodes[1] = [otherNode,abs(otherNode.y-self.y)]
        if otherNodePosRelative==2:
            self.otherNodes[2] = [otherNode,abs(otherNode.x-self.x)]
        if otherNodePosRelative==3:
            self.otherNodes[3] = [otherNode,abs(otherNode.y-self.y)]

class Path(Node): #basically a chain of nodes, alogirithm will creates different paths and at each step determine which path should be used
    def __init__(self,Parent,Child,pathlen):
        self.Parent = Parent
        self.Child = Child
        self.pathlen = pathlen
        self.heuristic = Child.heuristic
        self.Gscore = self.pathlen+self.heuristic
        self.prevPath = None
        


class PriorityQueue(Path): #creates a queue of paths, used to grab the node at the end of the path that gets us closest to the finish line
    def __init__(self):
        self.queue = []
    def isEmpty(self):
        return len(self.queue)==0
    def insert(self,data):
        self.queue.append(data) #data is a path
    def extract(self): #returns  node with highest heuristic
        try:
            max_val_inddx = 0 
            for i in range(len(self.queue)):
                if self.queue[i].Gscore< self.queue[max_val_inddx].Gscore:
                    max_val_inddx = i
            item = self.queue[max_val_inddx]
            del self.queue[max_val_inddx]
            return item
        except IndexError:
            print()
            exit()
    
def countNone(aList): #function to check how sides of a node is blocked no neighbour on a side means that in the neighbourArray, it is None
    counter = 0
    for i in range(len(aList)):
        if aList[i]==None:
            counter+=1
    return counter

def isAChild(curNode,listOfPaths): #checks if the current node the algortihm is looking at has already been reached by other paths
    for i in range(len(listOfPaths.queue)):
        if curNode == listOfPaths.queue[i].Child:
            return i
    return False

#checks if the current node is a child for any node in a particular path, used on closed path with the end node.
# if the end node is a child of the closed list, then the maze has been solved
def isAAChild(curNode,listOfPaths): 
    for i in range(len(listOfPaths)):
        if curNode == listOfPaths[i].Child:
            return i
    return False


# for maze generation, detemines which cell the maze creator should move to 
def chosenKey(curPos,grid,key):
    xsize = (grid.xMazeSize-1)-2
    ysize = (grid.yMazeSize-1)-2
    #start by assuming you can go to any cells around you horizontally and vertically
    moves = [0,1,2,3]
    # we don't wanna backtrack, if key is move forward, then we remove the option to move backwards
    if key == 0:
        moves.remove(1)
    elif key == 1:
        moves.remove(0)
    elif key == 2:
        moves.remove(3)
    elif key == 3:
        moves.remove(2)
    #grabbing the x and y positon of the current point
    x = getX(curPos.x)
    y = getY(curPos.y)
    #checking if the maze creator cell is at a boundary, removes the option to move thorugh the border
    if (x)==0:
        if 2 in moves:
            moves.remove(2)
    elif (x)==xsize:
        if 3 in moves:
            moves.remove(3)
    if (y)==0:
        if 0 in moves:
            moves.remove(0)
    elif (y)==ysize:
        if 1 in moves:
            moves.remove(1)
    #this code usually prevents the maze maker cell from passing into a cell its already been,
    #however a small chance has been provided allowing it to do so, this results in a more difficult maze
    # with more than one way to solve it and the potential for loops
    if y!=0:
        if grid.Mazze[y+1-2][x+1].visited == 1: #up
                if 0 in moves and random.random()<0.99:
                    moves.remove(0)
    if y!=(ysize):
        if grid.Mazze[y+1+2][x+1].visited == 1: #down
            if 1 in moves and random.random()<0.99:
                moves.remove(1)
    if x!=0:
        if grid.Mazze[y+1][x+1-2].visited == 1: #left
            if 2 in moves and random.random()<0.98:
                moves.remove(2)
    if x!=xsize:
        if grid.Mazze[y+1][x+1+2].visited == 1: #right
            if 3 in moves and random.random()<0.95:
                moves.remove(3)
    if len(moves)==0:
        return None
    if len(moves)==1:
        return moves[0]
    return moves[random.randint(0,(len(moves)-1))]

#function to move the maze maker cell
def move(curPos,grid,direction):
    grid.cellsToReDraw.append([getY(curPos.y)+1,getX(curPos.x)+1])
    if direction == 0:#curses.KEY_UP:
            curPos.y -= 2*size
            grid.Mazze[getY(curPos.y)+1+1][getX(curPos.x)+1].color = white
            grid.cellsToReDraw.append([getY(curPos.y)+1+1,getX(curPos.x)+1])
    elif direction == 1:#curses.KEY_DOWN:
            curPos.y += 2*size
            grid.Mazze[getY(curPos.y)+1-1][getX(curPos.x)+1].color = white
            grid.cellsToReDraw.append([getY(curPos.y)+1-1,getX(curPos.x)+1])
    elif direction == 2:#curses.KEY_LEFT:
            curPos.x -= 2*size
            grid.Mazze[getY(curPos.y)+1][getX(curPos.x)+1+1].color = white
            grid.cellsToReDraw.append([getY(curPos.y)+1,getX(curPos.x)+1+1])
    elif direction == 3:#curses.KEY_RIGHT:
            curPos.x += 2*size
            grid.Mazze[getY(curPos.y)+1][getX(curPos.x)+1-1].color = white
            grid.cellsToReDraw.append([getY(curPos.y)+1,getX(curPos.x)+1-1])

#tells what type of interesting cell a node is, eg Left L, right L, four way intersection T intersection, blind end ect
def check_conditions(grid, j, i):
    if grid.Mazze[j][i-1].color == white and grid.Mazze[j-1][i].color ==  black   and grid.Mazze[j][i+1].color ==  black   and grid.Mazze[j+1][i].color ==  white :
        return [True,5]
    if grid.Mazze[j][i-1].color == black and grid.Mazze[j-1][i].color ==  black  and grid.Mazze[j][i+1].color ==  white  and grid.Mazze[j+1][i].color ==  white :
        return [True,6]
    if grid.Mazze[j][i-1].color == white and grid.Mazze[j-1][i].color ==  white  and grid.Mazze[j][i+1].color ==  black  and grid.Mazze[j+1][i].color ==  black :
        return [True,8]
    if grid.Mazze[j][i-1].color == black and grid.Mazze[j-1][i].color ==  white  and grid.Mazze[j][i+1].color ==  white  and grid.Mazze[j+1][i].color ==  black :
        return [True,7]
    if grid.Mazze[j][i-1].color == black and grid.Mazze[j-1][i].color ==  black  and grid.Mazze[j][i+1].color ==  black  and grid.Mazze[j+1][i].color ==  white :
        return [True,12]
    if grid.Mazze[j][i-1].color ==  black  and grid.Mazze[j-1][i].color ==  black  and grid.Mazze[j][i+1].color ==  white  and grid.Mazze[j+1][i].color ==  black :
        return [True,9]
    if grid.Mazze[j][i-1].color ==  black  and grid.Mazze[j-1][i].color ==  white  and grid.Mazze[j][i+1].color ==  black  and grid.Mazze[j+1][i].color ==  black :
        return [True,10]
    if grid.Mazze[j][i-1].color ==  white  and grid.Mazze[j-1][i].color ==  black  and grid.Mazze[j][i+1].color ==  black  and grid.Mazze[j+1][i].color ==  black :
        return [True,11]
    if grid.Mazze[j][i-1].color ==  black  and grid.Mazze[j-1][i].color ==  white  and grid.Mazze[j][i+1].color ==  white  and grid.Mazze[j+1][i].color ==  white :
        return [True,2]
    if grid.Mazze[j][i-1].color ==  white  and grid.Mazze[j-1][i].color ==  black  and grid.Mazze[j][i+1].color ==  white  and grid.Mazze[j+1][i].color ==  white :
        return [True,1]
    if grid.Mazze[j][i-1].color ==  white  and grid.Mazze[j-1][i].color ==  white  and grid.Mazze[j][i+1].color ==  black  and grid.Mazze[j+1][i].color ==  white :
        return [True,4]
    if grid.Mazze[j][i-1].color ==  white  and grid.Mazze[j-1][i].color ==  white  and grid.Mazze[j][i+1].color ==  white  and grid.Mazze[j+1][i].color ==  black :
        return [True,3]
    if grid.Mazze[j][i-1].color ==  white  and grid.Mazze[j-1][i].color ==  white  and grid.Mazze[j][i+1].color ==  white  and grid.Mazze[j+1][i].color ==  white :
        return [True,13]
    return [False,-1]

#function for drawing the path once we solve the maze
def fill_cells_between(grid, start, end):
    x1, y1 = start
    x2, y2 = end
    if x1 == x2:  # Same x values, fill in y values
        for y in range(min(y1, y2), max(y1, y2) + 1):
            grid.Mazze[y][x1].color = red
    elif y1 == y2:  # Same y values, fill in x values
        for x in range(min(x1, x2), max(x1, x2) + 1):
            grid.Mazze[y1][x].color = red
#function to make the blue-red gradient effect of the path
def colorMap(ydata,yMazeSize):
    return int((255/yMazeSize)*ydata)

#makes the solver cell move towards its current destination node
def get_to_cell(grid,curPosi,destination):
    x1,y1 = curPosi
    x2,y2 = destination

    if curPosi!=destination:
        if (x1==x2):
            if (y1>y2):
                y1-=1
            elif (y1<y2):
                y1+=1
        elif (y1 == y2):
            if (x1>x2):
                x1-=1
            elif (x1<x2):
                x1+=1
        grid.Mazze[y1][x1].color = (colorMap(y1,grid.yMazeSize),0,255-colorMap(y1,grid.yMazeSize))
        grid.cellsToReDraw.append([y1,x1])
        grid.redrawCells()
        return [[x1,y1],1]
    else:
        return [[x1,y1],0]


running = True
runningrunning = True
# debugging: jjk=0
while runningrunning:
    guh = []
    key=0
    grid = Maze(random.randint(5,100),random.randint(5,50))
    curPos = Cell(50+size,50+size,blue)
    curPos.visitedCell = Stack()
    curPos.visitedCell.push([getX(50+size),getY(50+size)])
    screen = pygame.display.set_mode((grid.screen_width, grid.screen_height))
    grid.flipVisited([getX(50+size),getY(50+size)])
    grid.drawMazeCells()
    curPos.drawCell()
    pygame.display.flip()
    clock = pygame.time.Clock()
    FPS=240
    running = True
    #portion of code for generating maze
    while running and runningrunning: 
        for event in pygame.event.get():
            if event.type==pygame.QUIT:
                running = False
                runningrunning=False
                break
        while curPos.visitedCell.empty()==False:
            for event in pygame.event.get():
                if event.type==pygame.QUIT:
                    running = False
                    runningrunning=False
                    break
            grid.redrawCells()
            curPos.drawCell()
            pygame.display.flip()
            if curPos.visitedCell.empty()==False:
                prevX = curPos.x
                prevY = curPos.y
                grid.Mazze[getY(prevY) + 1][getX(prevX) + 1].color = white
                grid.cellsToReDraw.append([getY(prevY) + 1, getX(prevX) + 1])
                newCell = curPos.visitedCell.pop()
                curPos.x,curPos.y = invGetX(newCell[0]),invGetY(newCell[1])
                key = chosenKey(curPos,grid,key)
                if key != None:
                    curPos.visitedCell.push([getX(curPos.x),getY(curPos.y)])
                    move(curPos,grid,key)
                    grid.flipVisited([getX(curPos.x)+1,getY(curPos.y)+1]) 
                    curPos.visitedCell.push([getX(curPos.x),getY(curPos.y)])
            clock.tick(FPS)

        #Portion of code for generating the simplified node maze + hardcoding the 2nd last position of the maze maker to be white, otherwise it would be blue :p
        grid.Mazze[getY(prevY) + 1][getX(prevX) + 1].color = white
        grid.cellsToReDraw.append([getY(prevY) + 1, getX(prevX) + 1])
        curPos.x = grid.screen_width-50-size
        curPos.y = grid.screen_height-50-size
        xlen,ylen = getX(curPos.x),getY(curPos.y)
        guh = 0
        prevNode =None
        NodeMazzze = [[None]*grid.xMazeSize for _ in range(grid.yMazeSize)]

        for j in range(1,grid.yMazeSize-1):
            for i in range(1,grid.xMazeSize-1):
                if grid.Mazze[j][i].color!=black:
                    juh = check_conditions(grid,j,i)
                    if juh[0]:
                        NodeMazzze[j][i] = Node(grid,i,j,juh[1])
                        if (prevNode!=None) and (NodeMazzze[j][i].y == prevNode.y):
                            if (Node.lookingLeftMatrix[NodeMazzze[j][i].nodeType-1][prevNode.nodeType-1]==1):
                                NodeMazzze[j][i].addOtherNode(prevNode,0)
                                prevNode.addOtherNode(NodeMazzze[j][i],2)

                        guh+=1
                        prevNode = NodeMazzze[j][i] 
        prevNode=None
        for i in range(1,grid.xMazeSize-1):
            for j in range(1,grid.yMazeSize-1):
                if NodeMazzze[j][i] !=  None and (NodeMazzze[j][i].nodeType!= 9 and NodeMazzze!=11):
                    if (prevNode!=None) and (NodeMazzze[j][i].x == prevNode.x):
                        if (Node.lookingDownMatrix[prevNode.nodeType-1][NodeMazzze[j][i].nodeType-1]==1):
                            NodeMazzze[j][i].addOtherNode(prevNode,1)
                            prevNode.addOtherNode(NodeMazzze[j][i],3)
                    prevNode = NodeMazzze[j][i]

        #code for solving the maze
        closedList = [] #list of paths
        openList = PriorityQueue()
        contini = 0
        start = Path(NodeMazzze[grid.yMazeSize-2][grid.xMazeSize-2],NodeMazzze[grid.yMazeSize-2][grid.xMazeSize-2],0)
        openList.insert(start)
        while contini == 0:
            for event in pygame.event.get():
                if event.type==pygame.QUIT:
                    running = False
                    runningrunning=False
                    contini=1
                    break

            currentNode = openList.extract() #current path we are looking at
            closedList.append(currentNode)
            for stuff in currentNode.Child.otherNodes:
                if stuff!=None:
                    if isAAChild(stuff[0],closedList)==False:
                        inList = isAChild(stuff[0],openList)
                        if inList==False: #if it isn't in the open list
                            guh = Path(currentNode.Child,stuff[0],currentNode.pathlen+stuff[1])
                            guh.prevPath = currentNode
                            openList.insert(guh)
                        else: #if it is in the open queue
                            if (openList.queue[inList].pathlen>(currentNode.pathlen+stuff[1])):
                                openList.queue[inList].Parent = currentNode
                                openList.queue[inList].pathlen = currentNode.pathlen+stuff[1]
            if closedList[-1].Child == NodeMazzze[1][1]:
                contini=1

        #making the final list of nodes we need to visit to solve the maze
        nodesToVisit = []
        curPath = closedList[-1]
        while curPath.prevPath!=None:
            nodesToVisit.append(curPath.Child)
            curPath = curPath.prevPath
        
        #code for drawing in the colored path that solves the maze
        grid.redrawCells()
        pygame.display.flip()
        currentLocation = [grid.xMazeSize-2,grid.yMazeSize-2]
        catch = [currentLocation,1]
        while running:  
            for event in pygame.event.get():
                if event.type==pygame.QUIT:
                    running = False
                    runningrunning=False
                    break
            grid.Mazze[grid.yMazeSize-2][grid.xMazeSize-2].color = (colorMap(grid.yMazeSize-2,grid.yMazeSize),0,255-colorMap(grid.yMazeSize-2,grid.yMazeSize))
            grid.cellsToReDraw.append([curPath.Child.y,curPath.Child.x])
            for i in range(len(nodesToVisit)-1,-1,-1):
                while catch[1]==1:
                    catch = get_to_cell(grid,currentLocation,[nodesToVisit[i].x,nodesToVisit[i].y])
                    currentLocation = catch[0]
                    pygame.display.flip()
                    clock.tick(100)
                catch[1] = 1
            running = False

pygame.quit()
sys.exit()

#left top right down

