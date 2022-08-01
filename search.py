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
from util import Stack, Queue, PriorityQueue
import imp
import sys

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

#estrutura de auxílio para guardar os dados dos nós
#informação de estado, ação, parent
class Node:
    def __init__(self,state,action='',cost=0,parent=None):
        self.state = state
        self.action = action
        if parent:
            self.path_cost = parent.path_cost + cost
        else:
            self.path_cost = 0
        self.parent = parent

#concentre-se aqui
#busca em profundidade
def depthFirstSearch(problem: SearchProblem):  
    #pega o estado inicial do pacman
    root = Node(problem.getStartState())
    #lista de nós visitados
    visited = []
    #pilha pra controlar o backtracking
    stack = Stack()

    #enquanto o nó não é o objetivo
    while not problem.isGoalState(root.state):
        #insere o nó como visitado
        visited.append(root.state)
        #getSuccessors sucessor, ação para chegar lá, e custo getSuccessors
        for state,action,cost in problem.getSuccessors(root.state):
            #cria o nó, com a ação e o custo
            child = Node(state,action,cost,root)
            #se ainda não foi visitado vai colocar na pilha
            if not child.state in visited:
                #insere o nó no topo da pilha
                stack.push(child)
        #se trava em uma parede remove pra voltar um nó e testar o outro lado
        root = stack.pop()

    #se root state é o objetivo ele vai colocar na solução o caminho
    solution = []
    while root.parent:
        solution.insert(0,root.action)
        root = root.parent

    return solution

#largura
def breadthFirstSearch(problem: SearchProblem):
    #pega o estado inicial do pacman
    root = Node(problem.getStartState())

    #cria a fila exploração
    explored = Queue()
    #cria a fila fronteira de nós filhos
    fronteir = Queue()

    #se não é o objetivo vai inserir na 'fronteira'
    if not problem.isGoalState(root.state):
        fronteir.push(root)

    goal = None
    #enquanto tem nó na fronteira vai explorar
    while not (fronteir.isEmpty()):
        #remove da pilha fronteira
        node = fronteir.pop()
        #coloca na pilha de explorado
        explored.push(node.state)

        #se o novo for o objetivo para
        if problem.isGoalState(node.state):
            goal = node
            break
       
        #retorna pra fronteirStates os dados 
        fronteirStates = map(lambda x: x.state, fronteir.list)
        
        #pega os filhos do estado e olha
        for state,action,cost in problem.getSuccessors(node.state):
            #cria o nó, com a ação e o custo
            child = Node(state,action,cost,node)
            #se eles não foram explorados ou visitados coloca na fronteira
            if not (child.state in explored.list or child.state in fronteirStates):
                fronteir.push(child)

    #quando a fronteir ta vazia, chegou no objetivo
    #insere na solução
    solution = []
    while goal and goal.parent:
        solution.insert(0,goal.action)
        goal = goal.parent
    
    return solution

def uniformCostSearch(problem: SearchProblem):   
    #criando o nó, explorados e fronteira
    root = Node(problem.getStartState())
    explored = Queue()
    fronteir = PriorityQueue()
    
    # verifica se o começo ja é objetivo e insere na fronteira
    if not problem.isGoalState(root.state):
        fronteir.push(root,0)

    
    goal = None
    
    #se ha nós na fronteira remove de fronteira e insere em explorado
    while not (fronteir.isEmpty()):
        node = fronteir.pop()
        explored.push(node.state)

        #se é o objetivo para
        if problem.isGoalState(node.state):
            goal = node
            break
        
        #retorna pra fronteirStates os dados 
        fronteirStates = map(lambda x: x[2].state, fronteir.heap)

        #olhando a fronteira 
        for state,action,cost in problem.getSuccessors(node.state):
            child = Node(state,action,cost,node)

            #se o nó não está na fronteira nem explorado
            if not (child.state in explored.list or child.state in fronteirStates ):
                #coloca na fronteira
                fronteir.push(child,child.path_cost)
            #senão
            elif child.state in fronteirStates:
                idx = fronteirStates.index(child.state)
                if fronteir.heap[idx][0] > child.path_cost:
                    fronteir.push(child,child.path_cost)


    solution = []
    while goal and goal.parent:
        solution.insert(0,goal.action)
        goal = goal.parent
        
    return solution    


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    #pega e inicia o Nó, e cria explorados e fronteira
    root = Node(problem.getStartState())
    explored = Queue()
    fronteir = PriorityQueue()

    #se não é o objetivo pega a heurística e coloca na fronteira
    if not problem.isGoalState(root.state):
        h = heuristic(root.state, problem)
        fronteir.push(root,h)

    goal = None

    #enquanto tem nó fronteira tira da lista de nó e coloca nos explorados
    while not (fronteir.isEmpty()):
        node = fronteir.pop()
        explored.push(node.state)

        #se achou objetivo para
        if problem.isGoalState(node.state):
            goal = node
            break
        

        fronteirStates = map(lambda x: x[2].state, fronteir.heap)
        
        #percorrendo os seguintes
        for state,action,cost in problem.getSuccessors(node.state):
            child = Node(state,action,cost,node)
            h = heuristic(child.state, problem)

            #se não explorou ainda nem inseriu
            if not (child.state in explored.list or child.state in fronteirStates ):
                #insere na fronteira
                fronteir.push(child,child.path_cost+h)
            #senão 
            elif child.state in fronteirStates:
                idx = fronteirStates.index(child.state)
                if fronteir.heap[idx][0] > child.path_cost+h:
                    fronteir.push(child,child.path_cost+h)
    
    #inserir nas soluções
    solution = []
    while goal and goal.parent:
        solution.insert(0,goal.action)
        goal = goal.parent
        
    return solution

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
