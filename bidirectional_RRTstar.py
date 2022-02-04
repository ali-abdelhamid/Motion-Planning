import pygame
import math
import random
import time

#This class is for the visualization element (front end)
class RRTMap:
    def __init__(self,start,goal,MapDim,obsdim,obsnum):
        self.start = start
        self.goal = goal
        self.MapDim = MapDim
        self.MapH,self.MapW = self.MapDim

        #defining pygame window settings
        pygame.display.set_caption('RRT Path Planner')
        self.map = pygame.display.set_mode((self.MapW,self.MapH))
        self.map.fill((255,255,255))
        self.nodeRad = 3
        self.nodeThick = 0
        self.edgeThick = 1

        #defining obstacles
        self.obs = []
        self.obsdim = obsdim
        self.obsnum = obsnum

        #defining colors
        self.Grey = (70,70,70)
        self.Blue = (0,0,255)
        self.Green = (0,255,0)
        self.Red = (255,0,0)
        self.White = (255,255,255)

    def drawMap(self,obs):
        pygame.draw.circle(self.map,self.Green,self.start,self.nodeRad+5,0)
        pygame.draw.circle(self.map,self.Red,self.goal,self.nodeRad+5,0)
        self.drawObs(obs)
        

    def draw_path(self,path):
        for node in path:
            pygame.draw.circle(self.map, self.Red, node, self.nodeRad+3,0)

    def drawObs(self,obs):
        obs_list = obs.copy()
        while (len(obs_list)>0):
            obstacle = obs_list.pop(0)
            pygame.draw.rect(self.map,self.Grey,obstacle)


#This class is for defining and editing nodes and edges (backend)
class RRTGraph:
    def __init__(self,start,goal,MapDim,obsdim,obsnum):
        (x,y) = start
        self.start = start
        self.goal = goal
        self.goalFlag = False #set to true once goal is reached
        self.MapDim = MapDim
        self.MapH,self.MapW = self.MapDim

        #declaring empty lists for storing x,y coordinate and parent information for the nodes
        self.x = []
        self.y = []
        self.parent = []
        #initializing tree (at the start, the lists only contain the start node which has a parent of iteslf (0))
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)
        #defining obstacles in the workspace
        self.obs = []
        self.obsdim = obsdim
        self.obsnum = obsnum
        #define path and instantaneous goal state
        self.goalstate = None
        self.path = []


    def makeRandomRect(self):
        cornerx=int(random.uniform(0,self.MapW-self.obsdim))
        cornery=int(random.uniform(0,self.MapH-self.obsdim))
        return (cornerx,cornery)

    def makeObs(self):
        obs = []
        for i in range(0,self.obsnum):
            rectangle = None
            startgoal = True
            #keep re-trying to make rectangular obstacle as long as start and goal nodes are not within that rectangle
            while startgoal:
                upper = self.makeRandomRect()
                rectangle = pygame.Rect(upper,(self.obsdim,self.obsdim))
                if rectangle.collidepoint(self.start) or rectangle.collidepoint(self.goal):
                    startgoal = True
                else:
                    startgoal = False
            obs.append(rectangle)
        self.obs = obs.copy()
        return obs

    def add_node(self,n,x,y):
        self.x.insert(n,x)
        self.y.append(y)

    def remove_node(self,n):
        self.x.pop(n)
        self.y.pop(n)

    def add_edge(self,parent,child):
        self.parent.insert(child,parent)

    def remove_edge(self,n):
        self.parent.pop(n)

    def total_nodes(self):
        return len(self.x)

    def distance(self,n1,n2):
        (x1,y1) = (self.x[n1],self.y[n1])
        (x2,y2) = (self.x[n2],self.y[n2])
        px = (float(x1)-float(x2))**2
        py = (float(y1)-float(y2))**2
        #Euclidean Distance
        return (px+py)**(0.5)

    #randomly generate x and y coordinates (within MapDim)
    def sample_env(self):
        x = int(random.uniform(0,self.MapW))
        y = int(random.uniform(0,self.MapH))
        return x,y

    #measures distance from new (random) node to every other node in the tree
    def nearest(self,n):
        dmin = self.distance(0,n)
        nnear = 0
        for i in range(0,n):
            #looping through all n nodes to find the node with lowest dmin (nearest)
            if self.distance(i,n)<dmin:
                dmin = self.distance(i,n)
                nnear = i
        return nnear

    #is the newly generated random node in C_free?
    def isFree(self):
        n = self.total_nodes()-1
        (x,y) = (self.x[n],self.y[n])
        obst = self.obs.copy()
        while len(obst)>0:
            rectangle = obst.pop(0)
            if rectangle.collidepoint(x,y):
                self.remove_node(n)
                return False
        return True

    #is the edge between two nodes crossing any obstacles?
    #this method returns True for collision path and False for free path
    def x_obs(self,x1,x2,y1,y2):
        obst = self.obs.copy()
        while len(obst)>0:
            rectangle = obst.pop(0)
            #using interpolation to check for 100 pts on the line for collision
            for i in range(0,301):
                u = i/300
                x = x1*u + x2*(1-u)
                y = y1*u + y2*(1-u)
                if rectangle.collidepoint(x,y):
                    return True
        return False

    def connect(self,n1,n2):
        (x1,y1) = (self.x[n1],self.y[n1])
        (x2,y2) = (self.x[n2],self.y[n2])
        if self.x_obs(x1,x2,y1,y2):
            self.remove_node(n2)
            return False
        else:
            self.add_edge(n1,n2)
            return True

    #this method creates a new node that is between the new (random) node and the nearest node
    def step(self,nnear,nrand,dmax=30):
        d=self.distance(nnear,nrand)
        if d>dmax:
            u=dmax/d
            (xnear,ynear)=(self.x[nnear],self.y[nnear])
            (xrand,yrand)=(self.x[nrand],self.y[nrand])

            #we use the arctan function and the equation of a line to define a point between nnear and nnrand
            (px,py) = (xrand-xnear,yrand-ynear)
            theta = math.atan2(py,px)
            (x,y) = (int(xnear+dmax*math.cos(theta)),
                     int(ynear+dmax*math.sin(theta)))
            self.remove_node(nrand)

            #we check if the goal is reachable within step distance from the new node
            if abs(x-self.goal[0]) <= dmax and abs(y-self.goal[1]) <= dmax:
                self.add_node(nrand, self.goal[0], self.goal[1])
                self.goalstate = nrand
                self.goalFlag = True
            else:
                self.add_node(nrand,x,y)

    def path_2(self):
        if self.goalFlag:
            self.path=[]
            self.path.append(self.goalstate)
            newpos = self.parent[self.goalstate]
            while (newpos != 0):
                self.path.append(newpos)
                newpos = self.parent[newpos]
            self.path.append(0)
        return self.goalFlag

    def get_path(self):
        pathcoord = []
        for node in self.path:
            x,y = (self.x[node], self.y[node])
            pathcoord.append((x,y))
        return pathcoord
        
    #method for biasing the direction of the expansion (favoring nodes closer to the goal)
    def bias(self,ngoal):
        n=self.total_nodes()
        #add goal node to the tree and use it to find nearest node
        self.add_node(n,ngoal[0],ngoal[1])
        nnear = self.nearest(n)
        #take a step from nearest node to goal by creating new node between nnear and goal
        self.step(nnear,n)
        self.connect(nnear,n)
        return self.x,self.y,self.parent

    #mothod for "exploring" the map which is based on random direction 
    def expand(self):
        n = self.total_nodes()
        #notice how we use a random node for expand instead of the goal node)
        x,y = self.sample_env()
        self.add_node(n,x,y)
        if self.isFree():
            xnearest = self.nearest(n)
            self.step(xnearest,n)
            self.connect(xnearest,n)
        return self.x,self.y,self.parent
        




#MAIN SCRIPT
def main():
    dim = (500,500)
    start = (50,50)
    goal = (400,400)
    obsdim = 20
    obsnum = 50
    iteration = 1

    pygame.init()
    map = RRTMap(start,goal,dim,obsdim,obsnum)
    graph_1 = RRTGraph(start,goal,dim,obsdim,obsnum)
    graph_2 = RRTGraph(goal,start,dim,obsdim,obsnum)

    obs2 = graph_2.makeObs()
    graph_1.obs=graph_2.obs
    map.drawMap(obs2)

    X_1 = (0,0)
    X_2 = (20,20)
    Y_1 = (0,0)
    Y_2 = (20,20)
    while ((X_2[-1]-X_1[-1])>10 or (Y_2[-1]-Y_1[-1])>10):
        start_t = time.time()
        #every OTHER iteration we swap roles: start RRT expands to random and goal RRT expands to new latest start RRT node
        if iteration % 2 == 0:
            X_1, Y_1, Parent_1 = graph_1.expand()
            #index of [-1] returns the last node in the tree
            pygame.draw.circle(map.map,map.Blue, (X_1[-1], Y_1[-1]), map.nodeRad+2, 0)
            pygame.draw.line(map.map, map.Blue, (X_1[-1], Y_1[-1]), (X_1[Parent_1[-1]], Y_1[Parent_1[-1]]), map.edgeThick)

            X_new = (X_1[-1], Y_1[-1])

            #notice how in one hand of the iteration (under if) the goal RRT is biasing only towards X_new reached by the start RRT
            X_2, Y_2, Parent_2 = graph_2.bias(X_new)

            pygame.draw.circle(map.map,map.Red, (X_2[-1], Y_2[-1]), map.nodeRad+2, 0)
            pygame.draw.line(map.map, map.Red, (X_2[-1], Y_2[-1]), (X_2[Parent_2[-1]], Y_2[Parent_2[-1]]), map.edgeThick)

        #swapping roles per iteration
        else:
            X_2, Y_2, Parent_2 = graph_2.expand()
            
            pygame.draw.circle(map.map,map.Red, (X_2[-1], Y_2[-1]), map.nodeRad+2, 0)
            pygame.draw.line(map.map, map.Red, (X_2[-1], Y_2[-1]), (X_2[Parent_2[-1]], Y_2[Parent_2[-1]]), map.edgeThick)

            X_new = (X_2[-1], Y_2[-1])

            #notice how on the other hand of the iteration (under else) the start RRT is biasing only towards X_new reached by the goal RRT
            X_1, Y_1, Parent_1 = graph_1.bias(X_new)

            pygame.draw.circle(map.map,map.Blue, (X_1[-1], Y_1[-1]), map.nodeRad+2, 0)
            pygame.draw.line(map.map, map.Blue, (X_1[-1], Y_1[-1]), (X_1[Parent_1[-1]], Y_1[Parent_1[-1]]), map.edgeThick)

        if iteration % 1 == 0:
            pygame.display.update()
        iteration += 1

    end_t = time.time()
    delta_t = end_t - start_t
    print ('Time Taken:' + str (delta_t) + 'seconds')
    pygame.draw.line(map.map, map.Green, (X_1[-1], Y_1[-1]), (X_2[-1], Y_2[-1]), 4)
    #map.draw_path(graph_1.get_path())
    #map.draw_path(graph_2.get_path())
    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)

    run = True
    while run:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
    pygame.quit()

if __name__ == '__main__':
    main()









    
