'''
 IMPORTANT: Read through the code before beginning implementation!
 Your solution should fill in the various "TODO" items within this starter code.
'''
from __future__ import print_function
import math
import random
import copy

g_CYCLE_TIME = .100


# Default parameters will create a 4x4 grid to test with
g_MAP_SIZE_X = 2. # 2m wide
g_MAP_SIZE_Y = 1.5 # 1.5m tall
g_MAP_RESOLUTION_X = 0.5 # Each col represents 50cm
g_MAP_RESOLUTION_Y = 0.375 # Each row represents 37.5cm
g_NUM_X_CELLS = int(g_MAP_SIZE_X // g_MAP_RESOLUTION_X) # Number of columns in the grid map 
g_NUM_Y_CELLS = int(g_MAP_SIZE_Y // g_MAP_RESOLUTION_Y) # Number of rows in the grid map

# Map from Lab 4: values of 0 indicate free space, 1 indicates occupied space
g_WORLD_MAP = [0] * g_NUM_Y_CELLS*g_NUM_X_CELLS # Initialize graph (grid) as array

# Source and Destination (I,J) grid coordinates
g_dest_coordinates = (3,3)
g_src_coordinates = (0,0)


def create_test_map(map_array):
  # Takes an array representing a map of the world, copies it, and adds simulated obstacles
  num_cells = len(map_array)
  new_map = copy.copy(map_array) 
  # Add obstacles to up to sqrt(n) vertices of the map
  for i in range(int(math.sqrt(len(map_array)))):
    random_cell = random.randint(0, num_cells-1)
    new_map[random_cell] = 1
  
  return new_map

def vertex_index_to_ij(vertex_index):
  '''
  vertex_index: unique ID of graph vertex to be convered into grid coordinates
  Returns COL, ROW (x,y) coordinates in 2D grid
  '''
  global g_NUM_X_CELLS
  return vertex_index % g_NUM_X_CELLS, vertex_index // g_NUM_X_CELLS

def ij_to_vertex_index(i,j):
  '''
  i: Column of grid map (x)
  j: Row of grid map (y)

  returns integer 'vertex index'
  '''
  global g_NUM_X_CELLS
  return j*g_NUM_X_CELLS + i


def ij_coordinates_to_xy_coordinates(i,j):
  '''
  i: Column of grid map
  j: Row of grid map

  returns (X, Y) coordinates in meters at the center of grid cell (i,j)
  '''
  global g_MAP_RESOLUTION_X, g_MAP_RESOLUTION_Y
  return (i+0.5)*g_MAP_RESOLUTION_X, (j+0.5)*g_MAP_RESOLUTION_Y

def xy_coordinates_to_ij_coordinates(x,y):
  '''
  i: Column of grid map
  j: Row of grid map

  returns (i,j) grid cells corresponding to (x,y) coordinates in meters
  '''
  global g_MAP_RESOLUTION_X, g_MAP_RESOLUTION_Y
  return int(i // g_MAP_RESOLUTION_X), int(j // g_MAP_RESOLUTION_Y)

# **********************************
# *      Core Dijkstra Functions   *
# **********************************

def get_travel_cost(vertex_source, vertex_dest):
	global g_WORLD_MAP
  # Returns the cost of moving from vertex_source (int) to vertex_dest (int)
  # INSTRUCTIONS:
  	'''
      This function should return 1 if:
        vertex_source and vertex_dest are neighbors in a 4-connected grid (i.e., N,E,S,W of each other but not diagonal) 
        and neither is occupied in g_WORLD_MAP (i.e., g_WORLD_MAP isn't 1 for either)

      This function should return 1000 if:
        vertex_source corresponds to (i,j) coordinates outside the map
        vertex_dest corresponds to (i,j) coordinates outside the map
        vertex_source and vertex_dest are not adjacent to each other (i.e., more than 1 move away from each other)
  	'''
  	x_source, y_source = vertex_index_to_ij(vertex_source)
  	x_dest, y_dest = vertex_index_to_ij(vertex_dest)

  	# Check if vertices are witin map boundaries and unoccupied
  	if (x_source < 0 or y_source < 0 or x_dest < 0 or y_dest < 0):
  		return 1000
  	elif (x_source > (g_NUM_X_CELLS-1) or y_source > (g_NUM_Y_CELLS-1) or x_dest > (g_NUM_X_CELLS-1) or y_dest > (g_NUM_Y_CELLS-1)):
  		return 1000
  	elif (g_WORLD_MAP[vertex_source] == 1 or g_WORLD_MAP[vertex_dest] == 1):
  		return 1000

  	x_difference = abs(x_dest - x_source)
  	y_difference = abs(y_dest - y_source)

  	# Check if vertices are adjacent (already checked if within map and unoccupied)
  	if (x_difference == 1 and y_difference == 0):
  		cost_result = 1
  	elif (x_difference == 0 and y_difference == 1):
  		cost_result = 1
  	else:
  		cost_result = 1000
	
	return cost_result

def run_dijkstra(source_vertex):
 	'''
  source_vertex: vertex index to find all paths back to
  returns: 'prev' array from a completed Dijkstra's algorithm run

  Function to return an array of ints corresponding to the 'prev' variable in Dijkstra's algorithm
  The 'prev' array stores the next vertex on the best path back to source_vertex.
  Thus, the returned array prev can be treated as a lookup table:  prev[vertex_index] = next vertex index on the path back to source_vertex
 	'''  
	global g_NUM_X_CELLS, g_NUM_Y_CELLS, g_WORLD_MAP

	# Array mapping vertex_index to distance of shortest path from vertex_index to source_vertex.
	dist = [1000] * g_NUM_X_CELLS * g_NUM_Y_CELLS

	# Queue for identifying which vertices are up to still be explored:
	# Will contain tuples of (vertex_index, cost), sorted such that the min cost is first to be extracted (explore cheapest/most promising vertices first)
	Q_cost = []

	# Array of ints for storing the next step (vertex_index) on the shortest path back to source_vertex for each vertex in the graph
	prev = [-1] * g_NUM_X_CELLS*g_NUM_Y_CELLS

	for i, val in enumerate(g_WORLD_MAP):
		Q_cost.append([i,1000])
	dist[source_vertex] = 0
	Q_cost[source_vertex][1] = 0
	prev[source_vertex] = source_vertex

	while len(Q_cost) != 0:
		Q_cost.sort(key=lambda x: x[1])

		min_dist_vertex = Q_cost[0][0]
		Q_cost.pop(0)

		for i, val in enumerate(g_WORLD_MAP):
			if (get_travel_cost(min_dist_vertex,i) == 1): #Find every neighbor of 'min_dist_vertex'
				alt = dist[min_dist_vertex] + 1
				if (alt < dist[i]):
					for a, info in enumerate(Q_cost):
						if (info[0] == i):
							Q_cost[a][1] = alt
					dist[i] = alt
					prev[i] = min_dist_vertex

	return dist, prev


def reconstruct_path(prev, source_vertex, dest_vertex):
	'''
  Given a populated 'prev' array, a source vertex_index, and destination vertex_index,
  allocate and return an integer array populated with the path from source to destination.
  The first entry of your path should be source_vertex and the last entry should be the dest_vertex.
  If there is no path between source_vertex and dest_vertex, as indicated by hitting a '-1' on the
  path from dest to source, return an empty list.
	'''
	final_path = []

	# TODO: Insert your code here
	if (dest_vertex == source_vertex and prev[dest_vertex] != -1):
		return [dest_vertex]
	elif (prev[dest_vertex] != -1):
		while(prev[dest_vertex] != -1):
			final_path.append(dest_vertex)
			dest_vertex = prev[dest_vertex]
			if (dest_vertex == source_vertex):
				final_path.append(dest_vertex)
				break
			elif (prev[dest_vertex] == -1):
				return list()
	else:
		return list()

	# Return a reversed path list (so element 0 is the source)
	return final_path[::-1]


def render_map(map_array):
  	'''
  TODO-
    Display the map in the following format:
    Use " . " for free grid cells
    Use "[ ]" for occupied grid cells

    Example: 
    For g_WORLD_MAP = [0, 0, 1, 0,
                       0, 1, 1, 0, 
                       0, 0, 0, 0, 
                       0, 0, 0, 0]
    There are obstacles at (I,J) coordinates: [ (2,0), (1,1), (2,1) ]
    The map should render as:
      .  .  .  . 
      .  .  .  . 
      . [ ][ ] .
      .  . [ ] . 


    Make sure to display your map so that I,J coordinate (0,0) is in the bottom left.
    (To do this, you'll probably want to iterate from row 'J-1' to '0')
  	'''
  	global g_NUM_X_CELLS, g_NUM_Y_CELLS

	colWidth = 4
	max_column = g_NUM_X_CELLS-1
	max_row = g_NUM_Y_CELLS-1

	print()
	for j in range(max_row,-1,-1):
		for i in range(0,max_column+1):
			current_index = ij_to_vertex_index(i,j)
			if (map_array[current_index] == 0):
				print(".".center(colWidth), end="")
			else:
				print("[]".center(colWidth), end="")
		print()
	print()
	print()

def print_array_as_is(array):
	global g_NUM_X_CELLS, g_NUM_Y_CELLS

	colWidth = 7
	max_column = g_NUM_X_CELLS-1
	max_row = g_NUM_Y_CELLS-1

	for j in range(max_row,-1,-1):
		for i in range(0,max_column+1):
			current_index = ij_to_vertex_index(i,j)
			print(str(array[current_index]).center(colWidth), end="")
		print()
	print()
	print()

def main():
	global g_WORLD_MAP

  # TODO: Initialize a grid map to use for your test -- you may use create_test_map for this, or manually set one up with obstacles
	g_WORLD_MAP = create_test_map(g_WORLD_MAP)

  # Print array of world map for user to view
  	print(g_WORLD_MAP)

  # Use render_map to render your initialized obstacle map
	render_map(g_WORLD_MAP)

  # TODO: Find a path from the (I,J) coordinate pair in g_src_coordinates to the one in g_dest_coordinates using run_dijkstra and reconstruct_path
  	src_int = ij_to_vertex_index(g_src_coordinates[0],g_src_coordinates[1])
  	dest_int = ij_to_vertex_index(g_dest_coordinates[0],g_dest_coordinates[1])

	dist, prev = run_dijkstra(src_int)
	print("Dist:")
	print_array_as_is(dist)
	print("Prev: ")
	print_array_as_is(prev)

  	'''
  TODO-
    Display the final path in the following format:
    Source: (0,0)
    Goal: (3,1)
    0 -> 1 -> 2 -> 6 -> 7
  	'''
	shortest_path_array = reconstruct_path(prev,src_int,dest_int)
	#print(shortest_path_array)

	for i in range(0,len(shortest_path_array)):
		if (i == len(shortest_path_array)-1):
			print(str(shortest_path_array[i]))
		else:
			print(str(shortest_path_array[i]) + " -> ", end="")


if __name__ == "__main__":
  main()
