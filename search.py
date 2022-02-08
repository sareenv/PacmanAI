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

import queue
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
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    vnodes = []  # shows the visited nodes.
    current = problem.getStartState()

    stack = util.Stack()
    stack.push(([], current))

    while(stack.isEmpty() != True):
        moves, current = stack.pop()
        if(problem.isGoalState(current)):
            return moves
        if current not in vnodes:
            vnodes.append(current)
            for successor, action, _ in problem.getSuccessors(current):
                stack.push((moves + [action], successor))
    return None

    util.raiseNotDefined()


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    visited = []
    current = problem.getStartState()
    queue = util.Queue()
    queue.push(([], current))

    while(queue.isEmpty() != True):
        paths, node = queue.pop()
        if(problem.isGoalState(node)):
            return paths
        if node not in visited:
            visited.append(node)
            for neighbour in problem.getSuccessors(node):
                (successor, path, _) = neighbour
                queue.push((paths + [path], successor))
    return None

# Reference. - Learning the Search Algorithm.
# https://www.youtube.com/watch?v=dRMvK76xQJI - Resource used to learn the concept of UCS.
def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    visited = []
    current = problem.getStartState()
    pqueue = util.PriorityQueue()
    # push the path, node and cost
    pqueue.push(([], current, 0), 0)
    # rmwa
    while(pqueue.isEmpty() != True):
        (paths, node, cost) = pqueue.pop()
        if(problem.isGoalState(node)):
            return paths
        if(node not in visited):
            visited.append(node)
            neighbours = problem.getSuccessors(node)
            for neighbour in neighbours:
                (successor, path, scost) = neighbour
                new_cost = cost + scost
                priority = new_cost
                pqueue.push((paths + [path] ,successor, priority), priority)
    return None


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


# Do whole lot of less work because of the informed search ucs + heurstic function.
def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    visited = []
    current = problem.getStartState()
    initial_heuristic_val = heuristic(current, problem)
    estimatedPriority = 0 + initial_heuristic_val
    pqueue = util.PriorityQueue()
    pqueue.push(([], current, estimatedPriority), estimatedPriority)

    while(pqueue.isEmpty() != True):
        (paths, node, cost) = pqueue.pop()

        if(problem.isGoalState(node)):
            return paths

        if(node not in visited):
            visited.append(node)
            neighbours = problem.getSuccessors(node)
            for neighbour in neighbours:
                (successor, path, _) = neighbour
                initial_heuristic_val = heuristic(successor, problem)
                new_cost = problem.getCostOfActions(paths + [path]) + initial_heuristic_val
                priority = new_cost
                pqueue.push((paths + [path] ,successor, priority), priority)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
