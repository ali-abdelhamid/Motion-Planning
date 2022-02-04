import pygame
import math
import time
from queue import PriorityQueue

#Defining pygame window parameters
width = 1000
#note that having a square window allows us to build a square grid of the same number of rows and columns (square nodes)
window = pygame.display.set_mode((width, width))
pygame.display.set_caption("Path Finding Algorithm")

#Defining RGB colors for showing "state" of each node
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 200, 255)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (120, 0, 120)
ORANGE = (255, 165 ,0)
GREY = (160,160,160)

#defining nodes using a class with attributes that allow the program to know node location (row,col/x,y) node state (color) etc.
class Node:
	def __init__(self, row, col, width, total_rows):
		self.row = row
		self.col = col
		self.x = row * width
		self.y = col * width
		self.color = WHITE
		self.neighbors = []
		self.width = width
		self.total_rows = total_rows

        #This function will be called everytime our algorithm calculates the heuristic function from current node
	def get_pos(self):
		return self.row, self.col

        #The following methods check (using '==') whether or not a node is in a specific state based on its color
	def is_start(self):
		return self.color == ORANGE

	def is_end(self):
		return self.color == GREEN

	def is_closed(self):
		return self.color == RED

	def is_open(self):
		return self.color == BLUE

	def is_barrier(self):
		return self.color == BLACK

        #The following methods assign (using '=') a color to a node based on its state in the search or from the user input
	def reset(self):
		self.color = WHITE

	def make_start(self):
		self.color = ORANGE

	def make_end(self):
		self.color = GREEN

	def make_closed(self):
		self.color = RED

	def make_open(self):
		self.color = BLUE

	def make_barrier(self):
		self.color = BLACK

	def make_path(self):
		self.color = PURPLE

	def draw(self, window):
		pygame.draw.rect(window, self.color, (self.x, self.y, self.width, self.width))

        #The following method is used to assign neighbors for each node that is visited so long as it is not on the border
	#Note that in order for the point robot to be able to move in "all directions" we have to make each node 8 connected 
	def update_neighbors(self, grid):
		self.neighbors = []
		if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier(): # DOWN (1)
			self.neighbors.append(grid[self.row + 1][self.col])

		if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col - 1].is_barrier(): # SW (2)
			self.neighbors.append(grid[self.row + 1][self.col - 1])

		if self.row < self.total_rows - 1 and self.col < self.total_rows -1 and not grid[self.row + 1][self.col + 1].is_barrier(): # SE (3)
			self.neighbors.append(grid[self.row + 1][self.col + 1])

		if self.row > 0 and not grid[self.row - 1][self.col].is_barrier(): # UP (4)
			self.neighbors.append(grid[self.row - 1][self.col])

		if self.row > 0  and not grid[self.row - 1][self.col - 1].is_barrier(): # NW (5)
			self.neighbors.append(grid[self.row - 1][self.col - 1])

		if self.row > 0 and self.col < self.total_rows -1 and not grid[self.row - 1][self.col + 1].is_barrier(): # NE (6)
			self.neighbors.append(grid[self.row - 1][self.col + 1])

		if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier(): # RIGHT (7)
			self.neighbors.append(grid[self.row][self.col + 1])

		if self.col > 0 and not grid[self.row][self.col - 1].is_barrier(): # LEFT (8)
			self.neighbors.append(grid[self.row][self.col - 1])

	def __lt__(self, other):
		return False

#This method defines the Heuristic function that we use to estimate the distance from the current node to the goal
def h(p1, p2):
	x1, y1 = p1
	x2, y2 = p2
	return math.sqrt((x1 - x2)**2 + (y1 - y2)**2) #Euclidean Distance

#Once the end node is reached, this function is called to trace back (and paint) the (shortest) path that was taken from start to end
def reconstruct_path(came_from, current, draw):
	while current in came_from:
		current = came_from[current]
		current.make_path()
		draw()


def algorithm(draw, grid, start, end):
        #Start timer as soon as search algorithm is called
	T1 = time.perf_counter()

        #The count variable allows us to break ties between two nodes with identical f scores
        #by keeping track of which node was added first
	count = 0

        #Our open set consists of all nodes that we visited but have not yet explored the neighbors of
	#PriorityQueue allows us store nodes by order of f score (least score on top)
	#This way the "next" node to explore is always the one with the lowest f score
	open_set = PriorityQueue()
	#Each entry to the open set consists of the f score, the count, and the node location (row,col)
	open_set.put((0, count, start))

        #The came_from dictionary structure is used to keep track of what was the previous node that leads
	#to each current node which we will need for constructing the path once the end is reached
	came_from = {}

	#Assigning infinite values to unvisited nodes so that we only search nodes with defined scores first
	#(any defined score would be less than infinity so we always explore visited nodes first)
	#A node's scores become defined once we visit it
	g_score = {node: float("inf") for row in grid for node in row}
	f_score = {node: float("inf") for row in grid for node in row}
	f_score[start] = h(start.get_pos(), end.get_pos())
	g_score[start] = 0

        #open_set_track allows us to view the open set list since we are not able to "view" the PriorityQueue
	open_set_track = {start}

        #The algorithm runs as long as there are nodes in the open set i.e. as long as there are nodes to explore
	while not open_set.empty():
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit()

                #Current node is always the one with the smallest f score in the PriorityQueue
		current = open_set.get()[2]
		#We have to manually remove each node that we finished exploring from the open set tracker
		open_set_track.remove(current)

                #Terminate if current and end node are the same i.e. goal reached
		if current == end:
			reconstruct_path(came_from, end, draw)
			end.make_end()

                        #Stop timer as soon as end is reached
			T2 = time.perf_counter()
			tictoc=T2-T1
			print(tictoc)

                        #printing g-score of end node which is the minimum total cost to get to the end node
			print(g_score[current])
			return True

                #For traversing from one node to its neighbors, we assume all edge/path weights to be 1
		#Thus whenever we go from the current node to one of its neighbors, we add 1 to the g score
		for neighbor in current.neighbors:
			temp_g_score = g_score[current] + 1

                        #If the original g-score of the neighbor is larger than the g-score from the current node
			#this means that we just found a shorter way to get to that neighbor and so we overwrite the g score
			if temp_g_score < g_score[neighbor]:
				came_from[neighbor] = current
				g_score[neighbor] = temp_g_score
				f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), end.get_pos())
                                #Add neighbor to open set if it was not already there
				if neighbor not in open_set_track:
					count += 1
					open_set.put((f_score[neighbor], count, neighbor))
					open_set_track.add(neighbor)
					neighbor.make_open()

		draw()

                #Once the for loop is done going through all neighbors of a given node,
		#If the current node exits the for loop and is not the start node, then we make it closed
		if current != start:
			current.make_closed()

        #for when no path is found and all nodes in the open set have been searched
        
	return False
	
#This method builds the 2D array (grid) that is populated with rowsXrows nodes of size gap
def make_grid(rows, width):
	grid = []
        #The total width integer division by the total rows gives us us the size of equal gaps from border to border
	gap = width // rows
	for i in range(rows):
		grid.append([])
		for j in range(rows):
			node = Node(i, j, gap, rows)
			grid[i].append(node)

	return grid

#This method uses the same gap measurement to "skip" over the white node body and draw grid lines at the node border
def draw_grid(window, rows, width):
	gap = width // rows
	for i in range(rows):
		pygame.draw.line(window, GREY, (0, i * gap), (width, i * gap)) #horizontal lines
		for j in range(rows):
			pygame.draw.line(window, GREY, (j * gap, 0), (j * gap, width)) #vertical lines

#This method is to paint the entire window (window) white and all the border nodes black (barriers)
def draw(window, grid, rows, width):
	window.fill(WHITE)

	for row in grid:
		for node in row:
			node.draw(window)

                        #Drawing barriers at boundary so that workspace edges are not connected
			#Note that including this in the draw() method ensures that even if the user tries to erase
			#a barrier at the border, the algorithm will simply draw a barrier over it again
			if node.col == 0:
				node.make_barrier()
			if node.col == rows-1:
				node.make_barrier()
			if node.row == 0:
				node.make_barrier()
			if node.row == rows-1:
				node.make_barrier()

                        #Defining start and end nodes at opposite corners
			if node.col == 1 and node.row == 1:
				node.make_start()
			if node.col == rows-2 and node.row == rows-2:
				node.make_end()
        #Calling draw_grid function after grid array has been defined and populated with all open nodes and border (barrier) nodes
	draw_grid(window, rows, width)
	pygame.display.update()

#This method converts all continuous x,y values within a node to a single row,col pair of that node
def get_clicked_pos(pos, rows, width):
	gap = width // rows
	y, x = pos

        #For example, if the gap is 20 (20x20 size box of a node) then if the mouse cursor is at (10,10) the integer
	#division causes row and col to be 0 which means that we are at the top left node.
	row = y // gap
	col = x // gap

	return row, col


def main(window, width):
	rows = 50
	grid = make_grid(rows, width)

        #Defining start node in top left corner
	start = grid[1][1]
	start.make_start()

	#Defining end node in bottom right corner
	end = grid[rows-2][rows-2]
	end.make_end()

	run = True
	while run:
		draw(window, grid, rows, width)
		for event in pygame.event.get():
                        #If the exit icon is clicked on the pygame window, exit while loop
			if event.type == pygame.QUIT:
				run = False

                        #Left mouse click registration (enter start/end/barriers, in that order)
			if pygame.mouse.get_pressed()[0]:
				pos = pygame.mouse.get_pos()
				row, col = get_clicked_pos(pos, rows, width)
				node = grid[row][col]
				node.make_barrier()

                        #Right mouse click registration (undo an entry)
			elif pygame.mouse.get_pressed()[2]: 
				pos = pygame.mouse.get_pos()
				row, col = get_clicked_pos(pos, rows, width)
				node = grid[row][col]
				node.reset()
				if node == start:
					start = start
				elif node == end:
					end = end

                        #Start search when the space bar is pressed only if start and end nodes are defined
			if event.type == pygame.KEYDOWN:
				if event.key == pygame.K_SPACE and start and end:
					for row in grid:
						for node in row:
							node.update_neighbors(grid)

					algorithm(lambda: draw(window, grid, rows, width), grid, start, end)

                                #Clear window from all user input for barriers (that are not borders)
				if event.key == pygame.K_c:
					grid = make_grid(rows, width)
					#Defining start node in top left corner (after reset)
					start = grid[1][1]
					start.make_start()

					#Defining end node in bottom right corner (after reset)
					end = grid[rows-2][rows-2]
					end.make_end()

        #quit pygame if the while loop is terminated
	pygame.quit()

main(window, width)
