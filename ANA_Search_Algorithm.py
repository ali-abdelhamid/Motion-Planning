import sys
from PIL import Image
import copy
import math
from queue import PriorityQueue
import time 

'''
These variables are determined at runtime and should not be changed or mutated by you
'''
start = (0, 0)  # a single (x,y) tuple, representing the start position of the search algorithm
end = (0, 0)    # a single (x,y) tuple, representing the end position of the search algorithm
difficulty = "" # a string reference to the original import file

'''
These variables determine display coler, and can be changed by you, I guess
'''
NEON_GREEN = (0, 255, 0)
PURPLE = (85, 26, 139)
LIGHT_GRAY = (50, 50, 50)
DARK_GRAY = (100, 100, 100)

'''
These variables are determined and filled algorithmically, and are expected (and required) be mutated by you
'''
path = []       # an ordered list of (x,y) tuples, representing the path to traverse from start-->goal
expanded = {}   # a dictionary of (x,y) tuples, representing nodes that have been expanded
frontier = {}   # a dictionary of (x,y) tuples, representing nodes to expand to in the future

G = 1e15
E = 1e15

#Our entries for the open set (frontier) will be in the form of [e(s) g(s) (x,y)]
#and since the next ideal node to search would be the node with the HIGHEST e(s) value
#(i.e. smallest combination of cost so far and distance to go estimate) then we must reverse the queue
#we do so by multiplying the e(s) key by -1 which reverses the order (largest e(s) becomes smallest)
class Reverse_Queue(PriorityQueue):

	def put(self, en):
		en_2 = en[0] * -1, en[1], en[2]
		PriorityQueue.put(self, en_2)
	
	def get(self):
		en = PriorityQueue.get(self)
		en_2 = en[0] * -1, en[1], en[2]
		return en_2

#This method is to compute the h(s) function (straight line estimate)
def eucledian(p1, p2):
	(x1, y1) = p1
	(x2, y2) = p2
	return math.sqrt((x1 - x2)**2 + (y1 - y2)**2) #Euclidean Distance

#This method is to compute e(s)
def compute_e(G, g, h):
	e = (G - g)/(h + 1e-15) #To avoid crash when e(s) is computed for goal node
	return e

#Note that nodes are only added to the results list if they are not off grid and are not an obstacle
def traverse_nodes(map, size, node):
	x = node[0]
	y = node[1]
	results = []

	x_max = size[0]
	y_max = size[1]

        #Moving to the RIGHT 
	if (x+1 < x_max) and (map[x+1,y] != 1):
		results.append((x+1,y)) 
        #Moving to the LEFT
	if (y+1 < y_max) and (map[x,y+1] != 1): 
		results.append((x,y+1))
        #Moving UP
	if (y>=1) and (map[x,y-1] != 1): 
		results.append((x,y-1))
        #Moving DOWN 
	if (x>=1) and (map[x-1,y] != 1): 
		results.append((x-1,y))

	return results

 
def prune(front, G, goal):
	update_front = Reverse_Queue(0)

	while not front.empty():
		node = front.get()
		e_s = node[0]
		g_s = node[1]
		state = node[2]

		h_s = eucledian(state, goal)

		if g_s + h_s < G:
			new_e_s = compute_e(G, g_s, h_s)
			update_front.put((new_e_s, g_s, state))
		
	return update_front

def print_stats(time_taken, G, E, iterate):

	print ("Iteration Count: " + str(iterate))
	print ("Cost G           : " + str(G) + ' units')
	print ("Sub-optimality   : " + str(E) + ' units')
	print ("Time Taken       : " + str (time_taken) + ' sec')
	
	return time_taken

def ANA(map, size, start, goal):
	#setting initial G and E values to infinity
	global G, E

	#keeping track of total times the imporve_solution method finds a new (better) solution
	iterate = 0
	#computing initial h(s) and e(s)
	h_start = eucledian(start, goal)
	e = compute_e(G, 0, h_start)

	#inserting start node into open list
	front = Reverse_Queue(0) 
	front.put((e, 0, start))

	#dictionaries to keep track of node parent information and nodes already explored
	parent_linked = {}
	explored = {}
	parent_linked[start] = None
	explored[start] = 0

	time_step = 0

	#Keep running the imporve_solution method until all nodes from the fronteir list are explored
	while not front.empty():
		iterate += 1
		start_t = time.time()
		parent_linked, explored, front, G, E = improve_solution(parent_linked, explored, front, G, E, map, size, start, goal)
		end_t = time.time()
		delta_t = end_t - start_t
		
		time_step = print_stats(delta_t + time_step, G, E, iterate)
		front = prune(front, G, goal)
		
	path = []
	current_node = goal

	#Now that improve_solution has been called until no more nodes remain in the open list,
	#we can construct the path by looping through the parent_linked dictionary to backtrack to the start
	while current_node != start:
		path.append(current_node)
		current_node = parent_linked[current_node]
	path.append(start)
	path.reverse()	

	frontier = {}
	for f in front.queue:
		frontier[f[2]] = f[1]

	return path, explored, frontier, iterate

def improve_solution(parent_linked, explored, front, G, E, map, size, start, goal):
	#Note that in the ANA method, when the imporve_solution method is called:
        #the front input is already given as a reversed queue so we do not have to reverse it again
	while not front.empty():
		current_node = front.get()
		e_s = current_node[0]
		g_s = current_node[1]
		state = current_node[2]
		
		if e_s < E:
			E = e_s

		if state == goal:
			G = g_s
			break

		for successor in traverse_nodes(map, size, state):
			new_cost = explored[state] + 1
			if successor not in explored or new_cost < explored[successor] :
				explored[successor] = new_cost
				h_successor = eucledian(goal, successor)
				total_cost = new_cost + h_successor
				if total_cost < G:
					e_successor = compute_e(G, new_cost, h_successor)
					front.put((e_successor, new_cost, successor))
				parent_linked[successor] = state

	return parent_linked, explored, front, G, E

####

def search(map,size):
    
    global path, start, end, path, expanded, frontier
    
    """
    This function is meant to use the global variables [start, end, path, expanded, frontier] to search through the
    provided map.
    :param map: A '1-concept' PIL PixelAccess object to be searched. (basically a 2d boolean array)
    """

    # O is unoccupied (white); 1 is occupied (black)
    print ("pixel value at start point ", map[start[0], start[1]])
    print ("pixel value at end point ", map[end[0], end[1]])

    #calling ANA method which calls improve_solution method based on status of open list
    path, expanded, frontier, iterate = ANA(map, size, start, end)

    visualize_search("out.png") # see what your search has wrought (and maybe save your results)

def visualize_search(save_file="do_not_save.png"):
    """
    :param save_file: (optional) filename to save image to (no filename given means no save file)
    """
    im = Image.open(difficulty).convert("RGB")
    pixel_access = im.load()

    # draw start and end pixels
    pixel_access[start[0], start[1]] = NEON_GREEN
    pixel_access[end[0], end[1]] = NEON_GREEN

    # draw frontier pixels
    for pixel in frontier.keys():
        pixel_access[pixel[0], pixel[1]] = LIGHT_GRAY

    # draw expanded pixels
    for pixel in expanded.keys():
        pixel_access[pixel[0], pixel[1]] = DARK_GRAY
        
    # draw path pixels
    for pixel in path:
        pixel_access[pixel[0], pixel[1]] = PURPLE
        
    # draw start and end pixels
    pixel_access[start[0], start[1]] = NEON_GREEN
    pixel_access[end[0], end[1]] = NEON_GREEN

    # display and (maybe) save results
    im.show()
    if(save_file != "do_not_save.png"):
        im.save(save_file)

    im.close()


if __name__ == "__main__":
    # Throw Errors && Such
    # global difficulty, start, end
    #assert sys.version_info[0] == 2                                 # require python 2 (instead of python 3)
    assert len(sys.argv) == 2, "Incorrect Number of arguments"      # require difficulty input

    # Parse input arguments
    function_name = str(sys.argv[0])
    difficulty = str(sys.argv[1])
    print ("running " + function_name + " with " + difficulty + " difficulty.")

    # Hard code start and end positions of search for each difficulty level
    if difficulty == "trivial.gif":
        start = (8, 1)
        end = (20, 1)
    elif difficulty == "medium.gif":
        start = (8, 201)
        end = (110, 1)
    elif difficulty == "hard.gif":
        start = (10, 1)
        end = (401, 220)
    elif difficulty == "very_hard.gif":
        start = (1, 324)
        end = (580, 1)
    else:
        assert False, "Incorrect difficulty level provided"

    # Perform search on given image
    im = Image.open(difficulty)
    #Note how we also provide the image size as an input to the search method
    #this is so that we can make sure not to try and explore out of the image in the traverse method
    search(im.load(),im.size)
