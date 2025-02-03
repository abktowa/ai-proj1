package search;

/* DO NOT MODIFY THIS CLASS */

/**
This code is adapted from the Problem class in search.py in the AIMA Python implementation, which is published with the license below:

	The MIT License (MIT)

	Copyright (c) 2016 aima-python contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

The abstract class for a formal problem.  You should subclass this and
implement the method successor, and possibly the constructor, goal_test, and
path_cost. Then you will create instances of your subclass and solve them
with the various search functions.
*/

import java.util.ArrayList;

public abstract class Problem{
	protected State initial;
	protected State goal;
	
	/**
	The constructor specifies the initial state. Your subclass's constructor can add other arguments.
	@param initial The initial state of the problem
	*/
	public Problem(State initial){
		this.initial = initial;
		this.goal = null;
	}
	
	/** 
	The constructor specifies the initial state and a unique goal state. Your subclass's constructor can add other arguments.
	@param initial The initial state of the problem
	@param goal The goal state of the problem
	*/
	public Problem(State initial, State goal){
		this.initial = initial;
		this.goal = goal;
	}
	
	/**
	Return the initial state
	@return The problem's initial state
	*/
	public State getInitial(){
		return initial;
	}
		
	/**
	Given a state, return a set of (action, state) pairs reachable from this state.
	@param state A problem state
	@return A list of all (Action, State) pairs (as Tuples) that are reachable from the given state.
	*/
	public abstract ArrayList<Tuple> successor(State state);
	
	/**
	Returns true if the given state is a goal. The default method compares the state to this.goal, as specified in the constructor. Override this
	method if checking against a single goal is not enough.
	@param state A problem state
	@return True if the given state is a goal, false otherwise.
	*/
	public boolean goalTest(State state){
		return state.equals(this.goal);
	}
	
	/**
	Return the cost of a solution path that arrives at state2 from state1 via action, assuming cost c to get up to state1. If the problem
	is such that the path doesn't matter, this function will only look at state2.  If the path does matter, it will consider c and maybe state1
	and action. The default method costs 1 for every step in the path.
	@param c The path cost so far up to state1
	@param state1 The state prior to an action
	@param action The action taken
	@param state2 The state after taking the action
	@return The total path cost up to state2
	*/
	public double pathCost(double c, State state1, Action action, State state2){
		return c + 1;
	}
		
	/**
	Return the heuristic function value for a particular node. By default, returns 0. Override this if using informed (heuristic) search.
	@param node A search node
	@return The heuristic value for the state in that search node.
	*/
	public double h(Node node){
		return 0;
	}
}