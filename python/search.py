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
	'''YOUR CODE HERE'''
	pass
	
def depth_first_search(problem):
	'''YOUR CODE HERE'''
	pass

def uniform_cost_search(problem):
	'''YOUR CODE HERE'''
	pass
#______________________________________________________________________________
# Informed (Heuristic) Search

def astar_search(problem):
	'''YOUR CODE HERE'''
	pass

#______________________________________________________________________________
## Main

def main():
	'''REPLACE THIS CODE WITH CODE THAT RUNS THE PROGRAM SPECIFIED IN THE COMMAND ARGUMENTS'''

	print(sys.argv) # Prints the command line arguments. Note that the 0th element is the name of the file (search.py).
	

main()