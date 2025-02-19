"""
This code is adapted from search.py in the AIMA Python implementation, which is published with the license below:

	The MIT License (MIT)

	Copyright (c) 2016 aima-python contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Search (Chapters 3-4)

The way to use this code is to subclass Problem to create a class of problems,
then create problem instances and solve them with calls to the various search
functions."""

import sys
from collections import deque
import subway
import heapq
from subway import straight_line_distance


#______________________________________________________________________________

'''DO NOT MODIFY THIS CLASS'''

class Problem:
	"""The abstract class for a formal problem.  You should subclass this and
	implement the method successor, and possibly __init__, goal_test, and
	path_cost. Then you will create instances of your subclass and solve them
	with the various search functions."""
	
	

	def __init__(self, initial, goal=None):
		"""The constructor specifies the initial state, and possibly a goal
		state, if there is a unique goal.  Your subclass's constructor can add
		other arguments."""
		self.initial = initial; self.goal = goal
		
	def successor(self, state):
		"""Given a state, return a sequence of (action, state) pairs reachable
		from this state. If there are many successors, consider an iterator
		that yields the successors one at a time, rather than building them
		all at once. Iterators will work fine within the framework."""
		raise NotImplementedError("successor() must be implemented in subclass")
	
	def goal_test(self, state):
		"""Return True if the state is a goal. The default method compares the
		state to self.goal, as specified in the constructor. Implement this
		method if checking against a single self.goal is not enough."""
		return state == self.goal
	
	def path_cost(self, c, state1, action, state2):
		"""Return the cost of a solution path that arrives at state2 from
		state1 via action, assuming cost c to get up to state1. If the problem
		is such that the path doesn't matter, this function will only look at
		state2.  If the path does matter, it will consider c and maybe state1
		and action. The default method costs 1 for every step in the path."""
		return c + 1
		
	def h(self, node):
		"""Return the heuristic function value for a particular node. Implement
		this if using informed (heuristic) search."""
		return 0
#______________________________________________________________________________
class subway_problem(Problem): #Sub-Class of Problem 
	def __init__(self, initial, goal, subMap): #Additional parameter refers to subway map
		Problem.__init__(self, initial, goal)
		self.subMap = subMap
	
	def successor(self, state):
		return self.subMap.adjacent_stations(state)
	
	def goal_test(self, state):
		return state == self.goal

	def path_cost(self, c, state1, action, state2):
		chosenLinks = self.subMap.get_links_between(state1, state2)
		return c + next(chosenLinks).get_distance()

	def h(self, node):
   	 return straight_line_distance(node.state, self.goal)  # Use the standalone function


#______________________________________________________________________________
'''DO NOT MODIFY THIS CLASS'''

class Node:
	"""A node in a search tree. Contains a pointer to the parent (the node
	that this is a successor of) and to the actual state for this node. Note
	that if a state is arrived at by two paths, then there are two nodes with
	the same state.  Also includes the action that got us to this state, and
	the total path_cost (also known as g) to reach the node.  Other functions
	may add an f and h value. You will not need to
	subclass this class."""

	__nextID = 1

	def __init__(self, state, parent=None, action=None, path_cost=0):
		"Create a search tree Node, derived from a parent by an action."
		self.state = state
		self.parent = parent
		self.action = action
		self.path_cost = path_cost
		self.depth = 0
		self.id = Node.__nextID
		Node.__nextID += 1
		
		if parent:
			self.depth = parent.depth + 1
			
	def __str__(self):
		return "<Node " + str(self.state) + ">"
	
	def __repr__(self):
		return "<Node " + str(self.state) + ">"	
	
	def path(self):
		"Create a list of nodes from the root to this node."
		x, result = self, [self]
		while x.parent:
			result.append(x.parent)
			x = x.parent
		return result[::-1]

	def expand(self, problem):
		"Return a list of nodes reachable from this node. [Fig. 3.8]"
		return [Node(next, self, act,
					 problem.path_cost(self.path_cost, self.state, act, next))
				for (act, next) in problem.successor(self.state)]
	
	def __eq__(self, other):
		if isinstance(other, Node):
			return self.id == other.id
		return False
	
	def __lt__(self, other):
		if isinstance(other, Node):
			return self.id < other.id
		raise TypeError("\'<\' not supported between instances of Node and "+str(type(other)))
	
	def __hash__(self):
		return hash(self.id)


#______________________________________________________________________________
## Uninformed Search algorithms

'''DO NOT MODIFY THE HEADERS OF ANY OF THESE FUNCTIONS'''
def breadth_first_search(problem):
	"""Returns a tuple with the goal Node followed by an Integer with the amount of nodes visited
	(Returns None if a solution isn't found)"""
	# Setup
	queue = deque()
	visited = set()
	nodes_visited = 0

	# Add start node
	start = Node(problem.initial)
	queue.append(start)

	# Check if it's the goal
	if problem.goal_test(start.state):
		return (start, nodes_visited+1)
	
	while queue:
		current = queue.popleft()

		if current.state not in visited:
			# Visit
			visited.add(current.state)
			nodes_visited += 1

			if problem.goal_test(current.state):
				return (current, nodes_visited)
			
			# Insert neighbors
			neighbors = current.expand(problem)
			for neighbor in neighbors:
				if neighbor.state not in visited:
					queue.append(neighbor)

	return None
		
	
def depth_first_search(problem):
	"""Returns a tuple with the goal Node followed by an Integer with the amount of nodes visited
	(Returns None if a solution isn't found)"""
	# Setup
	stack = deque()
	visited = set()
	nodes_visited = 0

	# Add start node
	start = Node(problem.initial)
	stack.append(start)

	# Check if it's the goal
	if problem.goal_test(start.state):
		return (start, nodes_visited+1)

	while stack:
		current = stack.pop()

		if current.state not in visited:
			# Visit
			visited.add(current.state)
			nodes_visited += 1
			# Check if we found the solution
			if problem.goal_test(current.state):
				return (current, nodes_visited)
			
			# Insert neighbors
			neighbors = current.expand(problem)
			for neighbor in neighbors:
				if neighbor.state not in visited:
					stack.append(neighbor)

	return None

def uniform_cost_search(problem):
	# Initializing 
	queue = []
	visited = set()
	nodes_visited = 0
	opt_cost = {}

	start = Node(problem.initial)
	heapq.heappush(queue, (start.path_cost, start))
	opt_cost[start.state] = start.path_cost

	while queue: 
		_, current_node = heapq.heappop(queue)
		nodes_visited += 1

		if current_node.path_cost > opt_cost.get(current_node.state, float('inf')):
			continue

		if problem.goal_test(current_node.state):
			return (current_node, nodes_visited)
		
		visited.add(current_node.id)

		neighbors = current_node.expand(problem)
		for neighbor in neighbors:
			if neighbor.id not in visited:
				old_cost = opt_cost.get(neighbor.state, float('inf'))
				new_cost = neighbor.path_cost
				if new_cost < old_cost:
					opt_cost[neighbor.state] = new_cost
					heapq.heappush(queue, (new_cost, neighbor))

	return None
#______________________________________________________________________________
# Informed (Heuristic) Search

def astar_search(problem):
	queue = []
	visited = set()
	nodes_visited = 0
	opt_cost = {}

	start = Node(problem.initial)
	heapq.heappush(queue, (start.path_cost + problem.h(start), start))  
	opt_cost[start.state] = start.path_cost

	while queue:
		_, current_node = heapq.heappop(queue)  # Extract node with lowest f(n)
		nodes_visited += 1

		if current_node.state in visited:
			continue

		visited.add(current_node.state)

		if problem.goal_test(current_node.state):
			return (current_node, nodes_visited)

		for neighbor in current_node.expand(problem):
			if neighbor.state not in visited:
				old_cost = opt_cost.get(neighbor.state, float('inf'))
				if neighbor.path_cost < old_cost:
					opt_cost[neighbor.state] = neighbor.path_cost
					f_cost = neighbor.path_cost + problem.h(neighbor)  # Compute f(n)
					heapq.heappush(queue, (f_cost, neighbor))

	return None

#______________________________________________________________________________

## Output
def print_solution(solution):
	"The paramater is a tuple with the goal Node followed by an integer with the amount of nodes visited"
	print("Total cost: "+str(solution[0].path_cost))
	print("Number of search nodes visited: "+str(solution[1]))
	print("Final path: ")
	print_station_path(solution[0])

def print_station_path(node):
	stack = deque()

	while node.parent:
		stack.append(node.state)
		node = node.parent

	while stack:
		print(stack.pop())


## Main

def main():
	global cityMap
	# For debugging
	print(sys.argv) # Prints the command line arguments. Note that the 0th element is the name of the file (search.py).

	# Take input
	arg1 = sys.argv[1] # Options are "eight", "boston", and "london"
	algorithm = sys.argv[2] # Options are "dfs", "bfs", ucs, and "astar"
	initial = sys.argv[3] # Either the starting subway stop, or the starting position of the number tiles
	# Take more input
	goal = sys.argv[4] # the destination subway stop

	if arg1 == "eight":
		# Prepare the eight number puzzle

		# Prepare problem
		# problem = puzzle_problem()
		
		pass

	else:
		# Prepare subway search
		if arg1 == "boston":
			cityMap = subway.build_boston_map()
		elif arg1 == "london":
			cityMap = subway.build_london_map()
		
		initialCity = cityMap.get_station_by_name(initial)
		goalCity = cityMap.get_station_by_name(goal)
		# Get distance if it was provided
		if len(sys.argv) > 5:
			distance = sys.argv[5]
		else: 
			distance = 0

		# Prepare problem
		problem = subway_problem(initialCity, goalCity, cityMap)

	if algorithm == "bfs":
		print("Running BFS")
		print_solution(breadth_first_search(problem))
	elif algorithm == "dfs":
		print("Running DFS")
		print_solution(depth_first_search(problem))
	elif algorithm == "ucs":
		print("Running Uniformed Cost Search.")
		print_solution(uniform_cost_search(problem))
	elif algorithm == "astar":
		print("Running A*")
		print_solution(astar_search(problem))
	else:
		print(f"Unrecognized algorithm: {algorithm}")

main()