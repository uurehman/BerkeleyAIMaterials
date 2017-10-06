# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]


    
def depthFirstSearch(problem):

    explored = []        # List of already visisted nodes
    action_list = []    # List of actions taken to get to the current node
    initial = problem.getStartState()   # Starting state of the problem
    frontier=util.Stack()

    
    frontier.push((initial, action_list))
    
    while frontier: 
        
        nodeCoordinate, actions = frontier.pop() 

        if not nodeCoordinate in explored:
            explored.append(nodeCoordinate)
            if problem.isGoalState(nodeCoordinate):
                return actions
            successors = problem.getSuccessors(nodeCoordinate)
            for successor in successors:
                coordinate, direction, cost = successor
                newActions = actions + [direction]
                
                frontier.push((coordinate, newActions))

    return []

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    explored=[]  #list for visited nodes
    action_list=[] #required action to get current node
    initial=problem.getStartState() #initial state
    frontier=util.Queue()   #DFS so we use queue

    frontier.push((initial,action_list)) #push a tuple in queue
    while frontier:     #check if frontier(queue) is not empty
        nodeCoordinate,action=frontier.pop()     #pop the tuple from frontier(queue) and save values in identifiers
        if not nodeCoordinate in explored:
            explored.append(nodeCoordinate)      #check if current node not in explored list ,add it
            if problem.isGoalState(nodeCoordinate):       #check if current node is goal ,return all action taken to reach the goal
            
                return action
            neighbours=problem.getSuccessors(nodeCoordinate)  #get all associated node of current node
            for neighbour in neighbours:      #we access tuples with node complete info
                coordinate,direction,cost=neighbour    #store all neighbour node info in identifiers
                newAction=action+[direction]             #add direction of neighbour in action list
                frontier.push((coordinate,newAction))
    return []
   # util.raiseNotDefined()


def uniformCostSearch(problem):
    startState = problem.getStartState()     #get start point/node
    startCost = 0
   
    if problem.isGoalState(startState):    # check is it goal state or not
        return []
    myPrioirtyQueue = util.PriorityQueue()       # priority Queue declaration
    myPrioirtyQueue.push(startState, startCost)     # push in myPrioirtyQueue queue
    explored = {startState: [[], startCost]}
    
    while not myPrioirtyQueue.isEmpty():       #check if it is empty or not
        state = myPrioirtyQueue.pop()     # pop from myPrioirtyQueue
        if problem.isGoalState(state):   #check if it is goal state or not
            return explored[state][0]    #if  yes return from here
        if explored[state][0] == []:
            action = []
        else:
            action = explored[state][0]
        for coordinate, newAction, step in problem.getSuccessors(state):   # get its successor nodes
            childPath = action + [newAction]
            childCost = problem.getCostOfActions(childPath)
            if coordinate not in explored.keys():             # if it is not explored 
                myPrioirtyQueue.push(coordinate, childCost)         #push in myPrioirtyQueue
                explored[coordinate] = [childPath, childCost]   
            elif childCost < explored[coordinate][1]:
                explored[coordinate] = [childPath, childCost]
                myPrioirtyQueue.push(coordinate, childCost)
    
    return []

def UCS(problem, frontier, heuristic=None):

    explored = []        # List of already visisted nodes
    action_list = []    # List of actions taken to get to the current node
    initial = problem.getStartState()   # Starting state of the problem

    frontier.push((initial, action_list), heuristic(initial, problem))

    while frontier: 
        
        nodeCoordinate, actions = frontier.pop()
        
        if not nodeCoordinate in explored:
            explored.append(nodeCoordinate)
            if problem.isGoalState(nodeCoordinate):
                return actions
            successors = problem.getSuccessors(nodeCoordinate)
            for successor in successors:
                coordinate, direction, cost = successor
                newAction = actions + [direction]

                newCost = problem.getCostOfActions(newAction) + \
                               heuristic(coordinate, problem)
                frontier.push((coordinate, newAction), newCost)                  

    return []
    
def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    "Search the node that has the lowest combined cost and heuristic first."
    """
    Implement A* graph search in the empty function aStarSearch in search.py. A* takes a heuristic function as an argument. Heuristics take two arguments: a state in the search problem (the main argument), and the problem itself (for reference information). The nullHeuristic heuristic function in search.py is a trivial example.

    You can test your A* implementation on the original problem of finding a path through a maze to a fixed position using the Manhattan distance heuristic (implemented already as manhattanHeuristic in searchAgents.py).

    python pacman.py -l bigMaze -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic
    You should see that A* finds the optimal solution slightly faster than uniform cost search (about 549 vs. 620 search nodes expanded in our implementation, but ties in priority may make your numbers differ slightly). What happens on openMaze for the various search strategies?
    """
   
    return UCS(problem, util.PriorityQueue(), heuristic)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

